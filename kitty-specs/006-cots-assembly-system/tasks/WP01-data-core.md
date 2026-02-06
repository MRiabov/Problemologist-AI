---
work_package_id: "WP01"
title: "Data Core & Foundation"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004"]
---

# WP01: Data Core & Foundation

**Goal**: Establish the foundational data structures and database schema for the COTS Assembly System.

## Subtasks

### T001: Create project structure

**Purpose**: setup folders.
**Steps**:

1. Create `src/cots/` and `src/cots/database/` directories if they don't exist.
2. Add `__init__.py` files to make them packages.

### T002: Define Pydantic Models (`COTSItem`, `SearchQuery`)

**Purpose**: Define the data exchange format.
**File**: `src/cots/models.py`
**Content**:

- Implement `COTSItem` as per `data-model.md`:

  ```python
  class COTSItem(BaseModel):
      part_id: str
      name: str
      category: Literal["fastener", "motor", "gear", "bearing", "electronic"]
      unit_cost: float
      weight_g: float
      import_recipe: str  # build123d code snippet
      metadata: Dict[str, Any]
  ```

- Implement `SearchQuery` as per `data-model.md`.

### T003: Define SQLAlchemy ORM Models

**Purpose**: Define the persistence schema for `parts.db`.
**File**: `src/cots/database/models.py`
**Content**:

- Create a `Base` using `declarative_base()`.
- Create `COTSItemORM` class mapping to table `parts`.
- Columns should match `COTSItem`:
  - `part_id` (Primary Key, String)
  - `name` (String, indexed)
  - `category` (String, indexed)
  - `unit_cost` (Float)
  - `weight_g` (Float)
  - `import_recipe` (Text)
  - `metadata` (JSON) - use `sqlalchemy.types.JSON`

### T004: Create database initialization script

**Purpose**: Utility to create the database tables.
**File**: `src/cots/database/init.py`
**Content**:

- Function `init_db(db_path: str)`:
  - Create engine (`sqlite:///{db_path}`)
  - `Base.metadata.create_all(engine)`

## Validation

- [ ] Structure exists.
- [ ] `COTSItem` validation works.
- [ ] `init_db` creates a valid SQLite file with properties.

## Implementation Command

`spec-kitty implement WP01`
