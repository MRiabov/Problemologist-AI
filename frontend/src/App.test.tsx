import { render } from '@testing-library/react'
import { describe, it } from 'vitest'
import { MemoryRouter } from 'react-router-dom'
import App from './App'

describe('App', () => {
  it('renders without crashing', () => {
    render(
      <MemoryRouter>
        <App />
      </MemoryRouter>
    )
  })
})
