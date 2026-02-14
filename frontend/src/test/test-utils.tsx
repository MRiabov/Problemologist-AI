import React, { ReactElement } from 'react';
import { render, RenderOptions } from '@testing-library/react';
import { MemoryRouter } from 'react-router-dom';
import { ConnectionProvider } from '../context/ConnectionContext';
import { EpisodeProvider } from '../context/EpisodeContext';
import { ThemeProvider } from '../context/ThemeContext';

const AllTheProviders = ({ children }: { children: React.ReactNode }) => {
  return (
    <ThemeProvider>
      <ConnectionProvider>
        <EpisodeProvider>
          <MemoryRouter>
            {children}
          </MemoryRouter>
        </EpisodeProvider>
      </ConnectionProvider>
    </ThemeProvider>
  );
};

const customRender = (
  ui: ReactElement,
  options?: Omit<RenderOptions, 'wrapper'>,
) => render(ui, { wrapper: AllTheProviders, ...options });

export * from '@testing-library/react';
export { customRender as render };
