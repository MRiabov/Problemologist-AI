import { Link } from "react-router-dom";

export default function Sidebar() {
  return (
    <aside className="w-64 border-r bg-muted/40 hidden md:block min-h-screen">
      <div className="p-6">
        <h2 className="text-lg font-semibold tracking-tight">Agentic CAD</h2>
      </div>
      <nav className="px-4 pb-4">
        <ul className="space-y-1">
          <li>
            <Link to="/" className="block px-4 py-2 text-sm font-medium rounded-md hover:bg-accent hover:text-accent-foreground">
              Home
            </Link>
          </li>
          <li>
            <Link to="/wizard" className="block px-4 py-2 text-sm font-medium rounded-md hover:bg-accent hover:text-accent-foreground">
              Wizard
            </Link>
          </li>
        </ul>
      </nav>
    </aside>
  );
}
