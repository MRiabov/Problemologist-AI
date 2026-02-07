import '@testing-library/jest-dom';
import { expect, afterEach } from 'vitest';
import { cleanup } from '@testing-library/react';
import * as matchers from '@testing-library/jest-dom/matchers';
import { OpenAPI } from '../api/generated/core/OpenAPI';

// Mock window.location
if (typeof window !== 'undefined') {
  Object.defineProperty(window, 'location', {
    value: {
      origin: 'http://localhost:3000',
      pathname: '/',
    },
    writable: true,
  });
}

OpenAPI.BASE = 'http://localhost:3000';

expect.extend(matchers);

afterEach(() => {
  cleanup();
});