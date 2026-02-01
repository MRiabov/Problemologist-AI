from rich.console import Console
from rich.panel import Panel
from rich.markdown import Markdown
from langchain_core.messages import AIMessage, ToolMessage

console = Console()

def visualize_event(event):
    """
    Renders a single LangGraph event update to the console.
    """
    for node_name, updates in event.items():
        # Print Node Entry
        color = "blue"
        if node_name == "planner": color = "magenta"
        elif node_name == "actor": color = "cyan"
        elif node_name == "critic": color = "yellow"
        elif node_name == "tools": color = "green"
        
        console.print(Panel(f"Node Update: [bold]{node_name}[/bold]", border_style=color))
        
        # Display relevant updates
        if "messages" in updates:
            messages = updates["messages"]
            if not isinstance(messages, list):
                messages = [updates["messages"]]
                
            for msg in messages:
                if isinstance(msg, AIMessage):
                    content = msg.content
                    if content:
                        console.print(Panel(Markdown(content), title=f"🤖 {node_name.capitalize()}", border_style=color))
                    
                    if msg.tool_calls:
                        for tc in msg.tool_calls:
                            console.print(f"[bold {color}]🛠️  Tool Call: {tc['name']}[/bold {color}]")
                            
                elif isinstance(msg, ToolMessage):
                    console.print(Panel(msg.content[:500] + ("..." if len(msg.content) > 500 else ""), title=f"🔧 Tool Output ({msg.name})", border_style="dim white"))
        
        if "plan" in updates:
             console.print(Panel(Markdown(updates["plan"]), title="📋 Updated Plan", border_style="magenta"))


def visualize_stream(graph, inputs, config=None):
    """
    Streams the graph execution and renders updates to the console using Rich.
    """
    console.print(Panel("[bold green]Starting VLM CAD Agent...[/bold green]", title="System"))
    
    current_node = None
    
    # We use stream_mode="updates" to see the state changes from each node
    for event in graph.stream(inputs, config=config, stream_mode="updates"):
        for node_name, updates in event.items():
            
            # Print Node Entry
            color = "blue"
            if node_name == "planner": color = "magenta"
            elif node_name == "actor": color = "cyan"
            elif node_name == "critic": color = "yellow"
            elif node_name == "tools": color = "green"
            
            console.print(Panel(f"Exiting Node: [bold]{node_name}[/bold]", border_style=color))
            
            # Display relevant updates
            if "messages" in updates:
                messages = updates["messages"]
                if not isinstance(messages, list):
                    messages = [messages]
                    
                for msg in messages:
                    if isinstance(msg, AIMessage):
                        content = msg.content
                        if content:
                            console.print(Panel(Markdown(content), title=f"🤖 {node_name.capitalize()}", border_style=color))
                        
                        if msg.tool_calls:
                            for tc in msg.tool_calls:
                                console.print(f"[bold {color}]🛠️  Tool Call: {tc['name']}[/bold {color}]")
                                
                    elif isinstance(msg, ToolMessage):
                        console.print(Panel(msg.content[:500] + ("..." if len(msg.content) > 500 else ""), title=f"🔧 Tool Output ({msg.name})", border_style="dim white"))
            
            if "plan" in updates:
                 console.print(Panel(Markdown(updates["plan"]), title="📋 Updated Plan", border_style="magenta"))

    console.print(Panel("[bold green]Execution Complete[/bold green]", title="System"))