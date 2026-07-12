// Packed serial protocol types for the gimbal_driver <-> lower-machine link.
//
// Uplink uses TypedMessage + TypeID: lower machine -> gimbal_driver -> ROS topics.
// Downlink uses DownlinkTypeID-specific frames: ROS topics -> gimbal_driver -> lower machine.
// These are this repository's compact serial payloads; the lower machine is responsible for
// translating between them and full referee/chassis firmware protocols where noted.

#pragma once

#include <cstdint>
#include <cstddef>
#include <array>
#include <span>
#include <algorithm>
#include <ranges>
#include <type_traits>

namespace LangYa
{
    using AngleType = float;
    using Angle100Type = std::int16_t;

#pragma pack(push, 1)
    // Shared packed fragments. Do not change field order or bit width without
    // updating docs/sentry/embedded/serial_data_mapping.md and the lower machine.
    struct GimbalAnglesType
    {
        AngleType Yaw;
        AngleType Pitch;
    };

    struct VelocityType
    {
        std::int8_t X;
        std::int8_t Y;
    };

    struct UWBPositionType
    {
        std::int16_t X;
        std::int16_t Y;
    };

    /// @brief 1B fire/control state shared by TypeID=0 feedback and DownlinkTypeID=0x00 control.
    struct FireCodeType
    {
        /// @brief 开火状态，所有位反转表示开火 0b00 <-> 0b11
        std::uint8_t FireStatus : 2 = 0;

        /// @brief 电容状态， 00:不用 01:轻度使用 10:重度使用
        std::uint8_t CapState : 2 = 0;

        /// @brief 跟随模式，1 表示启用，0 表示禁用 follow the vel
        std::uint8_t FollowMode : 1 = 0;
        /// @brief 辅瞄模式，1 表示启用，0 表示禁用
        std::uint8_t AimMode : 1 = 0;

        /// @brief 小陀螺状态，共四档，0 表示无速，1 表示低速， 2 表示中速， 3 表示高速
        std::uint8_t Rotate : 2 = 0;

        /// @brief 翻转开火标志位
        void FlipFireStatus() noexcept { FireStatus = FireStatus == 0 ? 0b11 : 0b00; }
    };

    /// @brief RM2026 V2.0 4B sentry_cmd 布局，由独立下行 frame 映射到裁判 0x0301/0x0120。
    struct SentryCmdType
    {
        std::uint32_t ConfirmFreeRevive : 1 = 0;             // bit0
        std::uint32_t ConfirmImmediateRevive : 1 = 0;        // bit1
        std::uint32_t ExchangeProjectileAllowance : 11 = 0;  // bit2-12
        std::uint32_t RemoteProjectileExchangeCount : 4 = 0; // bit13-16
        std::uint32_t RemoteHpExchangeCount : 4 = 0;         // bit17-20
        std::uint32_t Posture : 3 = 0;                       // bit21-23, 1=进攻, 2=防御, 3=移动, 4~6=强化姿态
        std::uint32_t ConfirmEnergyActivate : 1 = 0;         // bit24
        std::uint32_t Reserved : 7 = 0;                      // bit25-31
    };
    static_assert(sizeof(SentryCmdType) == sizeof(std::uint32_t), "SentryCmdType must stay 4B");

    /// @brief TypeID=1 内的比赛摘要位域。前哨站血量是 6-bit 分度值，发布 ROS topic 时再 *25。
    /// @note 这里类型要求使用 @c std::uint16_t ，是因为在非 g++ 编译器上，此结构体的大小可能不符合预期
    struct GameCodeType
    {
        std::uint16_t IsGameBegin : 1 = 0;
        std::uint16_t HeroPrecaution : 1 = 0;
        std::uint16_t IsMyTeamRed : 1 = 0;
        std::uint16_t EnemyOutpostHealth : 6 = 60;
        std::uint16_t SelfOutpostHealth : 6 = 60;
        std::uint16_t IsReturnedHome : 1 = 0;
    };

    /// @brief TypeID=1 内的裁判场地事件数据 0x0101，当前按本仓库已接入的 32-bit 布局拆解。
    struct ExtEventDataType
    {
        std::uint32_t SelfSupplyStatus : 3 = 0;            // bit0-2
        std::uint32_t SelfSmallEnergyStatus : 2 = 0;       // bit3-4
        std::uint32_t SelfLargeEnergyStatus : 2 = 0;       // bit5-6
        std::uint32_t SelfCentralHighlandStatus : 2 = 0;   // bit7-8
        std::uint32_t SelfTrapezoidHighlandStatus : 2 = 0; // bit9-10
        std::uint32_t EnemyLastDartHitTime : 9 = 0;        // bit11-19
        std::uint32_t EnemyLastDartHitTarget : 3 = 0;      // bit20-22
        std::uint32_t CenterGainPointStatus : 2 = 0;       // bit23-24
        std::uint32_t SelfFortressGainPointStatus : 2 = 0; // bit25-26
        std::uint32_t SelfOutpostGainPointStatus : 2 = 0;  // bit27-28
        std::uint32_t SelfBaseGainPointStatus : 1 = 0;     // bit29
        std::uint32_t Reserved : 2 = 0;                    // bit30-31
    };

    // ==============================
    // Uplink: lower machine -> gimbal_driver
    // ==============================
    //
    // Frame layout: HeadFlag('!') + TypeID + 12B Data + Tail(0), total 15B.
    // TypeID is only for uplink payloads and is independent from downlink DownlinkTypeID.
    template<std::size_t TContentSize>
    struct TypedMessage
    {
        std::uint8_t HeadFlag{ '!' };
        std::uint8_t TypeID{ 0 };
        std::array<std::uint8_t, TContentSize> Data;
        std::uint8_t Tail{ 0 };

        template<typename T>
        requires std::is_trivially_copyable_v<T>
        void CopyDataTo(T* object) const noexcept
        {
            static_assert(sizeof(T) == TContentSize, "inconsistent size of data and object");
            std::ranges::copy(this->Data, reinterpret_cast<std::uint8_t*>(object));
        }

        template<typename T>
        requires std::is_trivially_assignable_v<T, const T>
        [[nodiscard]] T& GetDataAs() noexcept
        {
            static_assert(sizeof(T) == TContentSize, "inconsistent size of data and object");
            return *reinterpret_cast<T*>(this->Data.data());
        }

        template<typename T>
        requires std::is_trivially_assignable_v<T, const T>
        [[nodiscard]] const T& GetDataAs() const noexcept
        {
            static_assert(sizeof(T) == TContentSize, "inconsistent size of data and object");
            return *reinterpret_cast<const T*>(this->Data.data());
        }
    };

    /// @brief TypeID=0: 云台/火控/电容反馈，发布到 /ly/gimbal/angles、/ly/gimbal/firecode、/ly/gimbal/capV。
    struct GimbalData
    {
        static constexpr auto TypeID = 0;

        GimbalAnglesType GimbalAngles; // 8B, float yaw/pitch
        VelocityType Velocity;         // 2B, legacy raw velocity feedback
        FireCodeType FireCode;         // 1B, current fire/control state
        std::uint8_t CapV;
    };

    /// @brief TypeID=1: 比赛摘要/事件数据，发布到 /ly/game/all 及若干兼容 topic。
    struct GameData
    {
        static constexpr auto TypeID = 1;

        GameCodeType GameCode{};
        std::uint16_t AmmoLeft{};
        std::uint16_t TimeLeft{};
        std::uint16_t SelfHealth{};
        std::uint32_t ExtEventData{};
    };

    static_assert(sizeof(GimbalData) == sizeof(GameData), "inconsistent size of messages");

    /// @brief TypeID=2: 我方血量聚合，发布到 /ly/friend/hp 和 /ly/friend/base_hp。
    struct HealthMyselfData {
        static constexpr auto TypeID = 2;

        std::uint16_t HeroMyself{200};
        std::uint16_t EngineerMyself{};
        std::uint16_t Infantry1Myself{200};
        std::uint16_t Infantry2Myself{};
        std::uint16_t BaseMyself{};
        std::uint16_t SentryMyself{};
    };

    /// @brief TypeID=3: 敌方血量聚合，发布到 /ly/enemy/hp 和 /ly/enemy/base_hp。
    struct HealthEnemyData {
        static constexpr auto TypeID = 3;

        std::uint16_t HeroEnemy{};
        std::uint16_t EngineerEnemy{};
        std::uint16_t Infantry1Enemy{};
        std::uint16_t Infantry2Enemy{};
        std::uint16_t BaseEnemy{}; // 基地血量，2B 原始值
        std::uint16_t SentryEnemy{};
    };

    /// @brief 裁判 0x0204 buff 状态 + 部分 0x0209 RFID 状态的紧凑承载。
    struct BuffType{
        std::uint8_t reserve;
        std::uint8_t RecoveryBuff;
        std::uint8_t CoolingBuff;
        std::uint8_t DefenceBuff;
        std::uint8_t VulnerabilityBuff;
        std::uint16_t AttackBuff;
        std::uint8_t RemainingEnergy;
    };

    /// @brief TypeID=4: 增益与 RFID 低 32 位，发布到 /ly/team/buff 和 /ly/game/rfid。
    struct RFIDAndBuffData{
        static constexpr auto TypeID = 4;

        BuffType BuffStatus;
        // 对齐 RM2026 V1.3.0 的 0x0209 rfid_status（低 32 位，bit0-31）
        std::uint32_t RFIDStatus;
    };
    static_assert(sizeof(RFIDAndBuffData) == sizeof(GimbalData), "TypeID=4 payload must stay 12B");

    /// @brief TypeID=5 内的单车位置。CarId==7 时 Friend.X/Y 额外发布为 /ly/friend/uwb_pos。
    struct PositionType{
        uint8_t CarId; /// 兵种id
        int16_t X; /// 乘了100
        int16_t Y;
    };

    /// @brief TypeID=5: 友方/敌方位置 + legacy 弹速，发布到 /ly/position/data、/ly/friend/uwb_pos、/ly/bullet/speed。
    struct PositionData{
        static constexpr auto TypeID = 5;

        PositionType Friend;
        PositionType Enemy;
        uint16_t BulletSpeed; // 弹速，100倍
    };

    /// @brief TypeID=6: 底盘/UWB 回传，附带裁判 0x0003 offset8 damage_difference。
    struct ChassisData {
        static constexpr auto TypeID = 6;
        std::uint16_t UWBAngleYaw; // 2B
        std::int16_t DamageDifference; // 2B, 0x0003 offset8: 己方总伤害 - 对方总伤害
        // low16: 舵角当前角(int16, 解析后/10), high16: 底盘角速度(int16, 解析后/100)
        std::uint32_t ChassisPacked1;
        // low16: 底盘x方向速度(int16, 解析后/100), high16: 底盘y方向速度(int16, 解析后/100)
        std::uint32_t ChassisPacked2;
    };
    static_assert(sizeof(ChassisData) == sizeof(GimbalData), "TypeID=6 payload must stay 12B");

    /// @brief TypeID=7: 裁判 0x020D 哨兵状态 + 0x0207 初速度，发布到 /ly/game/sentry/info 和 /ly/game/bullet。
    struct SentryData {
        static constexpr auto TypeID = 7;
        std::uint32_t SentryInfo;        // 0x020D offset 0
        std::uint16_t SentryInfo2;       // 0x020D offset 4
        float BulletInitialSpeed;        // 0x0207 offset 3
        std::uint16_t Reserved{};
    };
    static_assert(sizeof(SentryData) == sizeof(GimbalData), "TypeID=7 payload must stay 12B");

    /// @brief TypeID=8: 裁判 0x0207 发射事件、0x0208 允许发弹量、0x0209 rfid_status_2。
    struct BulletDataAndRfid2 {
        static constexpr auto TypeID = 8;
        std::uint8_t BulletType;                     // 0x0207 offset 0
        std::uint8_t ShooterNumber;                  // 0x0207 offset 1
        std::uint8_t LaunchingFrequency;             // 0x0207 offset 2
        std::uint16_t ProjectileAllowance17mm;       // 0x0208 offset 0
        std::uint16_t ProjectileAllowance42mm;       // 0x0208 offset 2
        std::uint16_t RemainingGoldCoin;             // 0x0208 offset 4
        std::uint16_t ProjectileAllowanceFortress;   // 0x0208 offset 6
        std::uint8_t RfidStatus2;                    // 0x0209 offset 4
    };
    static_assert(sizeof(BulletDataAndRfid2) == sizeof(GimbalData), "TypeID=8 payload must stay 12B");

    /// @brief TypeID=9: 裁判 0x0303 选手端小地图交互数据 map_command_t，发布到 /ly/game/map_command。
    struct MapCommandData {
        static constexpr auto TypeID = 9;
        float TargetPositionX;       // 0x0303 offset 0, m; target robot mode should send 0
        float TargetPositionY;       // 0x0303 offset 4, m; target robot mode should send 0
        std::uint8_t CmdKeyboard;    // 0x0303 offset 8
        std::uint8_t TargetRobotId;  // 0x0303 offset 9; coordinate mode sends 0
        std::uint16_t CmdSource;     // 0x0303 offset 10
    };
    static_assert(sizeof(MapCommandData) == sizeof(GimbalData), "TypeID=9 payload must stay 12B");

    /// @brief TypeID=10: 裁判 0x020D sentry_info_3 + 0x0003 精确前哨站血量。
    struct SentryInfo3AndOutpostHpData {
        static constexpr auto TypeID = 10;
        std::uint64_t SentryInfo3;          // 0x020D offset 6: sentry_info_3
        std::uint16_t SelfOutpostHealth;    // 0x0003 offset 12: ally_outpost_HP
        std::uint16_t EnemyOutpostHealth;   // 0x0003 offset 16: enemy_outpost_HP
    };
    static_assert(sizeof(SentryInfo3AndOutpostHpData) == sizeof(GimbalData), "TypeID=10 payload must stay 12B");

    // ==============================
    // Downlink: gimbal_driver -> lower machine
    // ==============================
    //
    // DownlinkTypeID is independent from uplink TypeID, even though both occupy
    // byte1 of their own frame type. Frame size is selected by DownlinkTypeID.
    enum class DownlinkFrameType : std::uint8_t {
        Control = 0x00,
        SentryCommand = 0x01,
        MapPath = 0x02,
        CustomInfo = 0x03,
        SentryCoordinate = 0x04,
    };

    constexpr std::uint8_t ToRawDownlinkTypeID(DownlinkFrameType type_id) noexcept {
        return static_cast<std::uint8_t>(type_id);
    }

    /// @brief DownlinkTypeID=0x00: 主控制帧，由 /ly/control/angles、vel、firecode 组包。
    struct GimbalControlFrame
    {
        static constexpr auto FrameType = DownlinkFrameType::Control;
        static constexpr std::uint8_t DownlinkTypeIDValue = 0x00;

        std::uint8_t HeadFlag{ '!' };
        std::uint8_t DownlinkTypeID{ DownlinkTypeIDValue };
        VelocityType Velocity;         // /ly/control/vel
        GimbalAnglesType GimbalAngles; // /ly/control/angles
        FireCodeType FireCode;         // /ly/control/firecode
    };
    static_assert(
        GimbalControlFrame::DownlinkTypeIDValue == ToRawDownlinkTypeID(GimbalControlFrame::FrameType),
        "GimbalControlFrame DownlinkTypeIDValue must match DownlinkFrameType");
    static_assert(sizeof(GimbalControlFrame) == 13, "GimbalControlFrame must stay 13B");

    /// @brief DownlinkTypeID=0x01: 裁判 0x0120 sentry_cmd，来源 /ly/control/posture 或 /ly/control/sentry_cmd。
    struct SentryCommandFrame
    {
        static constexpr auto FrameType = DownlinkFrameType::SentryCommand;
        static constexpr std::uint8_t DownlinkTypeIDValue = 0x01;

        std::uint8_t HeadFlag{ '!' };
        std::uint8_t DownlinkTypeID{ DownlinkTypeIDValue };
        SentryCmdType SentryCmd;
    };
    static_assert(
        SentryCommandFrame::DownlinkTypeIDValue == ToRawDownlinkTypeID(SentryCommandFrame::FrameType),
        "SentryCommandFrame DownlinkTypeIDValue must match DownlinkFrameType");
    static_assert(sizeof(SentryCommandFrame) == 6, "SentryCommandFrame must stay 6B");

    /// @brief DownlinkTypeID=0x02: 裁判 0x0307 map_data_t，来源 /ly/control/map_path。
    struct MapPathFrame
    {
        static constexpr auto FrameType = DownlinkFrameType::MapPath;
        static constexpr std::uint8_t DownlinkTypeIDValue = 0x02;

        std::uint8_t HeadFlag{ '!' };
        std::uint8_t DownlinkTypeID{ DownlinkTypeIDValue };
        std::uint8_t Intention{ 0 };
        std::uint16_t StartPositionX_dm{ 0 };
        std::uint16_t StartPositionY_dm{ 0 };
        std::int8_t DeltaX_dm[49]{ 0 };
        std::int8_t DeltaY_dm[49]{ 0 };
        std::uint16_t SenderId{ 0 };
    };
    static_assert(
        MapPathFrame::DownlinkTypeIDValue == ToRawDownlinkTypeID(MapPathFrame::FrameType),
        "MapPathFrame DownlinkTypeIDValue must match DownlinkFrameType");
    static_assert(sizeof(MapPathFrame) == 107, "MapPathFrame must stay 107B");

    /// @brief DownlinkTypeID=0x03: 裁判 0x0308 custom_info_t，来源 /ly/control/custom_info。
    struct CustomInfoFrame
    {
        static constexpr auto FrameType = DownlinkFrameType::CustomInfo;
        static constexpr std::uint8_t DownlinkTypeIDValue = 0x03;

        std::uint8_t HeadFlag{ '!' };
        std::uint8_t DownlinkTypeID{ DownlinkTypeIDValue };
        std::uint16_t SenderId{ 0 };
        std::uint16_t ReceiverId{ 0 };
        std::uint8_t UserDataUtf16[30]{ 0 };
    };
    static_assert(
        CustomInfoFrame::DownlinkTypeIDValue == ToRawDownlinkTypeID(CustomInfoFrame::FrameType),
        "CustomInfoFrame DownlinkTypeIDValue must match DownlinkFrameType");
    static_assert(sizeof(CustomInfoFrame) == 36, "CustomInfoFrame must stay 36B");

    /// @brief DownlinkTypeID=0x04: BT 融合后的哨兵自身坐标，来源 /ly/bt/sentry_position。
    struct SentryCoordinateFrame
    {
        static constexpr auto FrameType = DownlinkFrameType::SentryCoordinate;
        static constexpr std::uint8_t DownlinkTypeIDValue = 0x04;

        std::uint8_t HeadFlag{ '!' };
        std::uint8_t DownlinkTypeID{ DownlinkTypeIDValue };
        std::int16_t X_cm{ 0 };
        std::int16_t Y_cm{ 0 };
        std::uint8_t Reserved[10]{ 0 };
        std::uint8_t CRC8{ 0 };
    };
    static_assert(
        SentryCoordinateFrame::DownlinkTypeIDValue == ToRawDownlinkTypeID(SentryCoordinateFrame::FrameType),
        "SentryCoordinateFrame DownlinkTypeIDValue must match DownlinkFrameType");
    static_assert(sizeof(SentryCoordinateFrame) == 17, "SentryCoordinateFrame must stay 17B");

#pragma pack(pop)
}
