import { render } from './test/test-utils'
import { describe, it } from 'vitest'
import App from './App'

describe('App', () => {
  it('renders without crashing', () => {
    render(<App />)
  })
})
