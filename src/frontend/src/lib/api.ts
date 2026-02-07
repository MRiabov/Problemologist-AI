import { OpenAPI } from '../api/core/OpenAPI';

// Configure the API client
OpenAPI.BASE = import.meta.env.VITE_API_URL || 'http://localhost:8000';

export { OpenAPI };
