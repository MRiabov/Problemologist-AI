import { Outlet } from "react-router-dom";
import Sidebar from "./Sidebar";

export default function AppLayout() {
  return (
    <div className="grid grid-cols-12 h-screen w-full overflow-hidden bg-background text-foreground">
      <div className="col-span-3 h-full overflow-hidden">
        <Sidebar />
      </div>
      <main className="col-span-9 flex flex-col min-w-0 overflow-hidden">
        <Outlet />
      </main>
    </div>
  );
}
