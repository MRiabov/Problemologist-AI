## 1. Solution Overview

Use a minimal engineering fixture so mention parsing can be evaluated without
extra geometry noise.

## 2. Parts List

- Single rigid part

## 3. Assembly Strategy

1. Create the rigid part.
2. Keep the solution deterministic and easy to validate.

## 4. Assumption Register

- Assumption: The planner relies on source-backed inputs that must be traceable.

## 5. Detailed Calculations

- CALC-001: The plan includes stable derivations rather than freeform guesses.

## 6. Critical Constraints / Operating Envelope

- Constraint: The mechanism must remain inside the derived operating limits.

## 7. Cost & Weight Budget

- Estimated unit cost: $8
- Estimated weight: 40g

## 8. Risk Assessment

- The main risk is overcomplicating the fixture and obscuring the core check.
