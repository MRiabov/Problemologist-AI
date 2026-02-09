from sqlalchemy import create_engine, inspect
import os

db_path = "problemologist.db"  # Default sqlite db
if os.path.exists(db_path):
    engine = create_engine(f"sqlite:///{db_path}")
    inspector = inspect(engine)
    columns = [col["name"] for col in inspector.get_columns("traces")]
    print(f"Traces columns: {columns}")
else:
    print("DB file not found")
