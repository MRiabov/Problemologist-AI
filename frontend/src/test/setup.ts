import '@testing-library/jest-dom';
import { expect, afterEach } from 'vitest';
import { cleanup } from '@testing-library/react';
import * as matchers from '@testing-library/jest-dom/matchers';
import { OpenAPI } from '../api/generated/core/OpenAPI';

OpenAPI.BASE = '';

expect.extend(matchers);

afterEach(() => {
  cleanup();
});