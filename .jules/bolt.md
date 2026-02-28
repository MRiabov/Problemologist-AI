## 2026-02-28 - Postgres connection limit exhaustion
**Learning:** Initializing an `AsyncConnectionPool` inside an `asynccontextmanager` generator runs per-request in web backends (e.g. LangGraph API requests). A pool initialized with `max_size=20` will immediately exhaust all available Postgres connections when several concurrent API calls hit that context manager.
**Action:** Always verify if database components managed by an async context manager in a persistence layer use a module-level cached singleton initialized behind an `asyncio.Lock()`.
