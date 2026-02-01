from pathlib import Path
from typing import List


class Workspace:
    def __init__(self, root_dir: str = ".workspace"):
        self.root_dir = Path(root_dir)
        self.root_dir.mkdir(parents=True, exist_ok=True)

    def _validate_path(self, path: str) -> Path:
        """Ensure path is within workspace to prevent traversal attacks."""
        full_path = (self.root_dir / path).resolve()
        # Security check: ensure the resolved path starts with the resolved root dir
        if not str(full_path).startswith(str(self.root_dir.resolve())):
            raise ValueError(f"Path {path} is outside workspace root {self.root_dir}")
        return full_path

    def read(self, path: str) -> str:
        """Read content from a file in the workspace."""
        full_path = self._validate_path(path)
        if not full_path.exists():
            return ""  # Or raise error? returning empty string is safer for "read if exists" logic
        return full_path.read_text(encoding="utf-8")

    def write(self, path: str, content: str) -> None:
        """Write content to a file in the workspace (overwrites)."""
        full_path = self._validate_path(path)
        full_path.parent.mkdir(parents=True, exist_ok=True)
        full_path.write_text(content, encoding="utf-8")

    def append(self, path: str, content: str) -> None:
        """Append content to a file in the workspace."""
        full_path = self._validate_path(path)
        full_path.parent.mkdir(parents=True, exist_ok=True)
        with open(full_path, "a", encoding="utf-8") as f:
            f.write(content)

    def list_files(self, pattern: str = "*", recursive: bool = False) -> List[str]:
        """List files in the workspace matching a pattern."""
        if recursive:
            return [
                str(p.relative_to(self.root_dir))
                for p in self.root_dir.rglob(pattern)
                if p.is_file()
            ]
        return [
            str(p.relative_to(self.root_dir))
            for p in self.root_dir.glob(pattern)
            if p.is_file()
        ]
