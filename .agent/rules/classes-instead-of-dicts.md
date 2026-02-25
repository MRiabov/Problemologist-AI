---
trigger: always_on
---

# Rule: Use Pydantic Models for Data Exchange

**Core Instruction**: ALWAYS prefer strict, typed `pydantic.BaseModel` classes over freeform `dict` objects for data exchange, function arguments, and return values.

## Rationale

- **Observability**: Structured schemas allow us to easily log and store execution data in our local observability database.
- **Hygiene**: Enforces types and validation at runtime, preventing silent data errors.
- **Interpretability**: explicit schemas make the code self-documenting.

## Specific Guidelines

1. **Data Transfer**: Define Pydantic models for any structured data passed between components.
2. **Persistence**: Use **SQLAlchemy** (not Pydantic) for defining persistent database schema checks/storage.
3. **Refactoring**: If you encounter code using `dict` for structured data, refactor it to use a Pydantic model.

## Additional strict typing guidelines

### Enums over strings

1. Enums (Uppercase) over strings where possible, including frontend and db.
2. `pathlib.Path` over string paths.
3. Enums or classes over substring matching.

### API calls and schema exchange (including tests

During API calls, including tests, always convert the JSON/YAML/XML to a type first, and only then user it or assert against it. The goal is to
