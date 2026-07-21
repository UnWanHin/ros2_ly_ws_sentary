# ProtectCastle RFID Stay Design

Updated: 2026-07-21

## Goal

Add `Tactical.ProtectCastle.StayWhenRfid` as a runtime YAML switch for the
formal regional decision path. When the referee confirms that the opponent
occupies the team's fortress gain point, the sentry must navigate to `Castle`
and, once arrived, remain there rather than selecting the surrounding Castle
search points or using navigation chase.

## Scope

This change applies only to the referee RFID/event source:

- master switch: `Tactical.ProtectCastle.Enable`
- source switch: `Tactical.ProtectCastle.RFID`
- new behaviour switch: `Tactical.ProtectCastle.StayWhenRfid`
- fresh referee condition: `self_fortress_gain_point_status == 2 || == 3`

`EnemyPos` remains an independent ProtectCastle source. Its current
navigation, search, and chase behaviour is not changed by this work.

## Behaviour

With all three YAML switches enabled and a fresh opponent-or-both fortress
occupation event:

1. The RegionalDefense RFID branch selects `Castle` as its only navigation
   candidate.
2. Before arrival, normal navigation carries the sentry to `Castle`.
3. At Castle arrival, navigation chase and chase velocity are suppressed.
4. Aim target selection, gimbal angle control, rotate, and fire behaviour
   continue normally; the new switch only prevents the chassis from leaving
   the gain point.

With `StayWhenRfid: false`, all existing RFID fortress behaviour remains
unchanged, including its surrounding-point search. With a stale event or an
event status of `0` or `1`, the stay lock is inactive.

## Interface

`Tactical.yaml` gains this additive, hot-editable field:

```yaml
Tactical:
  ProtectCastle:
    Enable: true
    RFID: true
    EnemyPos: true
    StayWhenRfid: true
```

The field defaults to `false` in the C++ configuration model to preserve
behaviour when an older parameter source does not provide it. The checked-in
formal `Tactical.yaml` explicitly sets it to `true`.

## Implementation Boundary

Keep policy predicates in `TacticalProtectionPolicy.hpp`, configuration
parsing in `Configuration.cpp`, and runtime decision enforcement in the
existing RegionalDefense/chase paths. Do not add topics, messages, launch
arguments, or modules.

## Verification

- Add unit coverage for the enabled/master/source/event predicate.
- Add tests proving an active stay lock only becomes effective after Castle
  arrival and leaves EnemyPos behaviour untouched.
- Extend the Tactical static-contract self-check for the new YAML and
  configuration key.
- Run the focused behavior_tree test target, `./scripts/selfcheck.sh sentry
  --static-only`, and `git diff --check`.
