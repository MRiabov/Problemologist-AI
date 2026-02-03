---
trigger: always_on
---

I prefer strict, typed Pydantic classes instead of freeform dicts for data exchange. This is because we can easily add observability to it, as we already store things to a local DB for observability, and also maintains code hygiene and interpretability.
If you need to add something to the DB for a persistent operation, we use SQLAlchemy for that.
