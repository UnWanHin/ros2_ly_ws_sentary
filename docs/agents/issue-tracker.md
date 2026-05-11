# Issue Tracker: GitHub

Issues, PRDs, and task breakdowns for this repo live in GitHub Issues for `HUSTLYRM/2026_sentry`.

Use the `gh` CLI from inside this clone so it can infer the repository from `git remote -v`.

## Conventions

- Create an issue with `gh issue create --title "..." --body "..."`.
- Use heredocs for multi-line issue or PRD bodies.
- Read an issue with `gh issue view <number> --comments`.
- List issues with `gh issue list --state open --json number,title,body,labels,comments`.
- Comment with `gh issue comment <number> --body "..."`.
- Apply or remove labels with `gh issue edit <number> --add-label "..."` and `--remove-label "..."`.
- Close issues with `gh issue close <number> --comment "..."`.

## Skill Behavior

When a skill says "publish to the issue tracker", create a GitHub issue.

When a skill says "fetch the relevant ticket", run `gh issue view <number> --comments`.
