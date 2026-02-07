import { useParams } from "react-router-dom"; export default function RunDetailPage() { const { id } = useParams(); return <div className="p-4"><h1>Run Detail: {id}</h1></div>; }
