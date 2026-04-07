# Git Agent

## Role Summary

The Git Agent is repository plumbing, not a model-facing workflow role.

## What It Owns

- git state synchronization side effects handled by the runtime

## What It Reads

- the workspace repository state

## Native Tool Surface

- none

## What Humans Must Tell It

- Nothing in prompt or skill revisions; this role is backend plumbing.
- If git behavior changes, update the runtime and filesystem contracts instead of inventing a prompt for this role.

## Acceptance Checklist

- The workspace git state is managed by the runtime.
- No agent prompt depends on this role for business logic.

## Related Skills

- none
