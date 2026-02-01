from rich.console import Console
from rich.panel import Panel
from rich.markdown import Markdown
from langchain_core.messages import AIMessage, ToolMessage

console = Console()


def visualize_node_entry(node_name: str):
    """Prints a header when entering a graph node."""
    console.print(f"\n[bold magenta]>>> Entering Node:[/bold magenta] [bold cyan]{node_name}[/bold cyan]")


def visualize_node_output(node_name: str, output: dict):
    """Prints the output of a graph node in a formatted panel."""
    if "messages" in output:
        last_msg = output["messages"][-1]
        
        if isinstance(last_msg, AIMessage):
            content = last_msg.content
            if last_msg.tool_calls:
                tools = ", ".join([tc["name"] for tc in last_msg.tool_calls])
                content += f"\n\n[bold yellow]Tool Calls:[/bold yellow] {tools}"
            
            console.print(Panel(
                Markdown(content) if content else "Empty AIMessage",
                title=f"Output from {node_name}",
                border_style="blue"
            ))
        elif isinstance(last_msg, ToolMessage):
            console.print(Panel(
                last_msg.content,
                title=f"Tool Output ({last_msg.name})",
                border_style="green"
            ))
    
    if "plan" in output and node_name == "planner":
        console.print(Panel(
            Markdown(output["plan"]),
            title="Updated Plan",
            border_style="yellow"
        ))


async def stream_graph_async(graph, inputs, config=None):
    """
    Streams graph updates asynchronously and visualizes them in the console.
    """
    async for event in graph.astream(inputs, config=config, stream_mode="updates"):
        for node_name, output in event.items():
            visualize_node_entry(node_name)
            visualize_node_output(node_name, output)


def stream_graph(graph, inputs, config=None):
    """
    Streams graph updates synchronously and visualizes them in the console.
    """
    for event in graph.stream(inputs, config=config, stream_mode="updates"):
        for node_name, output in event.items():
            visualize_node_entry(node_name)
            visualize_node_output(node_name, output)
