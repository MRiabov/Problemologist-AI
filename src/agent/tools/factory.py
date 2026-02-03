"""
Tool Factory Module.
Creates stateless tool instances bound to a specific runtime environment.
Replaces the global-state dependent tools from src.agent.tools.env.
"""

import asyncio
import difflib
import re
from typing import List, Optional

from langchain_core.tools import StructuredTool

from src.environment.runtime import ToolRuntime


def create_tools(runtime: ToolRuntime) -> List[StructuredTool]:
    """
    Creates a list of LangChain tools bound to the provided runtime.
    """

    async def write_file(content: str, path: str, mode: str = "overwrite") -> str:
        """
        Writes content to a specific file or appends to it.
        Use this to create scripts, update logs, or record journal entries.
        If path is 'journal.md', automatic timestamping is applied in append mode.

        Args:
            content: The text content to write.
            path: The path where the file should be saved (e.g., 'design.py', 'journal.md').
            mode: Either 'overwrite' or 'append' (default: 'overwrite').
        """
        return await asyncio.to_thread(runtime.write_file, content, path, mode)

    async def edit_file(path: str, find: str, replace: str) -> str:
        """
        Performs string replacement on the specified file.
        Use this to make targeted changes to existing files without rewriting them.

        Args:
            path: The path of the file to edit.
            find: The exact string to look for.
            replace: The string to replace it with.
        """
        return await asyncio.to_thread(runtime.edit_file, path, find, replace)

    async def view_file(path: str) -> str:
        """
        Reads the content of any file in the workspace or documentation.
        Use this to inspect scripts, skill files, or configuration.

        Args:
            path: Relative path to the file (e.g., 'design.py', 'docs/skills/build123d_cad_drafting_skill/SKILL.md').
        """
        return await asyncio.to_thread(runtime.view_file, path)

    async def run_command(command: str) -> str:
        """
        Executes a shell command inside the persistent sandbox environment.
        Use this to run your Python scripts (`python design.py`), list files (`ls`), or check syntax.
        You MUST have an active session for this to be effective.

        Args:
            command: The shell command to execute.
        """
        return await asyncio.to_thread(runtime.run_command, command)

    async def preview_design(path: str = "design.py") -> str:
        """
        Runs the current design script, exports a render, and returns the view.
        Use this to visually verify your geometry before submission.
        No reward is given for this action.

        Args:
            path: Path to the script to preview (default: design.py).
        """
        return await asyncio.to_thread(runtime.preview_design, path)

    async def check_manufacturability(
        design_file: str = "design.py", process: str = "cnc", quantity: int = 1
    ) -> dict:
        """
        Checks if the design in the specified file can be manufactured using the target process.
        Supported processes: 'cnc', 'injection_molding'.

        Args:
            design_file: The name of the script file to analyze.
            process: The manufacturing process to check against ('cnc' or 'injection_molding').
            quantity: Target production quantity (affects cost).
        """
        return await asyncio.to_thread(
            runtime.check_manufacturability, design_file, process, quantity
        )

    async def submit_design(control_path: str) -> str:
        """
        Runs the current design script, performs full Workbench validation, and returns final grades.
        Uses the script at control_path for motor logic (PID control, etc).
        This call marks the end of an attempt and triggers the full simulation.

        Args:
            control_path: Path to the controller script to be used in simulation.
        """
        return await asyncio.to_thread(runtime.submit_design, control_path)

    async def search_docs(query: str) -> str:
        """
        RAG retrieval from build123d and problemologist technical documentation.
        Use this to learn about CAD library syntax, specific functions, or problem scenarios.

        Args:
            query: The search query or question to look up.
        """
        return await asyncio.to_thread(runtime.search_docs, query)

    async def search_parts(query: str) -> str:
        """
        Search for COTS parts by name or ID. Returns a list of matches.
        Use this to find standard components like motors, bearings, or fasteners.

        Args:
            query: The search query or part name to look up.
        """
        return await asyncio.to_thread(runtime.search_parts, query)

    async def preview_part(part_id: str) -> str:
        """
        Get visual preview and details for a specific COTS part ID.
        Returns a description and a Python recipe to instantiate it.

        Args:
            part_id: The namespaces ID of the part (e.g., 'bd_warehouse:motor:Nema17').
        """
        return await asyncio.to_thread(runtime.preview_part, part_id)

    async def read_journal(topic: str = "") -> str:
        """
        Read the agent's journal (journal.md) to retrieve context or past memories.

        Args:
            topic: Optional topic or keyword to filter entries by.
        """
        content = await asyncio.to_thread(runtime.view_file, "journal.md")
        if not content or "Error:" in content:
             return "Journal is empty or not found."

        if topic:
            entries = re.findall(r"(## \[.*?\][\s\S]*?)(?=(?:## \[|$))", content)

            def is_match(entry_text: str, search_topic: str) -> bool:
                entry_lower = entry_text.lower()
                topic_lower = search_topic.lower()
                if topic_lower in entry_lower:
                    return True
                entry_words = re.findall(r"\w+", entry_lower)
                topic_words = re.findall(r"\w+", topic_lower)
                if not topic_words:
                    return False
                for t_word in topic_words:
                    matches = difflib.get_close_matches(
                        t_word, entry_words, n=1, cutoff=0.7
                    )
                    if not matches:
                        return False
                return True

            filtered_entries = [entry for entry in entries if is_match(entry, topic)]
            if not filtered_entries:
                return f"No journal entries found matching topic: '{topic}'."
            return "".join(filtered_entries).strip()

        return content

    async def lint_script(filename: str = "design.py") -> str:
        """
        Runs static analysis (Ruff, Pyrefly) on a script in the workspace.
        Use this to find syntax errors or potential bugs before running or submitting.

        Args:
            filename: The name of the script to lint (default: 'design.py').
        """
        return await asyncio.to_thread(runtime.lint_script, filename)

    async def list_skills() -> str:
        """
        Lists all available specialized skills.
        Use this to discover what knowledge categories are available.
        """
        return await asyncio.to_thread(runtime.list_skills)

    async def list_skill_files(skill_name: str) -> str:
        """
        Lists all files within a specialized skill folder, grouped by type.
        Use this to see the available reference documents or scripts for a skill.

        Args:
            skill_name: The name of the skill to inspect.
        """
        return await asyncio.to_thread(runtime.list_skill_files, skill_name)

    async def read_skill(
        skill_name: str, filename: str = "SKILL.md", resource_type: Optional[str] = None
    ) -> str:
        """
        Reads the content of a specialized skill.
        MANDATORY: You must read the 'build123d_cad_drafting_skill' before writing any build123d code.

        Args:
            skill_name: The name of the skill to read.
            filename: The filename (e.g., 'SKILL.md').
            resource_type: Optional. One of: 'scripts', 'references', 'assets'.
        """
        return await asyncio.to_thread(runtime.read_skill, skill_name, filename, resource_type)

    async def update_skill(
        skill_name: str,
        content: str,
        filename: str = "SKILL.md",
        resource_type: Optional[str] = None,
    ) -> str:
        """
        Updates or adds information to a specialized skill folder (e.g., 'build123d_cad_drafting_skill').
        Use this to capture new knowledge, patterns, or documentation discovered during task execution.

        Args:
            skill_name: The name of the skill to update.
            content: The Markdown or Python content to write.
            filename: The filename (e.g., 'SKILL.md', 'patterns.md', 'helper.py').
            resource_type: Mandatory if filename is not 'SKILL.md'. One of: 'scripts', 'references', 'assets'.
        """
        return await asyncio.to_thread(runtime.update_skill, skill_name, content, filename, resource_type)

    async def init_skill(skill_name: str) -> str:
        """
        Initializes a new skill directory with the canonical structure (SKILL.md, scripts/, references/, assets/).
        Use this when you want to create a new category of specialized knowledge.

        Args:
            skill_name: The name of the skill to create (hyphen-case, e.g., 'my-new-skill').
        """
        return await asyncio.to_thread(runtime.init_skill, skill_name)

    async def package_skill(skill_name: str) -> str:
        """
        Validates and packages a skill into a distributable .skill file.
        Use this when you have finished developing or updating a skill.

        Args:
            skill_name: The name of the skill to package.
        """
        return await asyncio.to_thread(runtime.package_skill, skill_name)

    async def run_skill_script(
        skill_name: str, script_name: str, arguments: str = ""
    ) -> str:
        """
        Executes a specialized script located within a skill's 'scripts' folder.
        Use this to perform deterministic tasks or fetch dynamic data defined by a skill.

        Args:
            skill_name: The name of the skill containing the script.
            script_name: The filename of the script (e.g., 'fetch_data.py').
            arguments: Optional string of arguments to pass to the script.
        """
        return await asyncio.to_thread(runtime.run_skill_script, skill_name, script_name, arguments)

    return [
        StructuredTool.from_function(write_file),
        StructuredTool.from_function(edit_file),
        StructuredTool.from_function(view_file),
        StructuredTool.from_function(run_command),
        StructuredTool.from_function(preview_design),
        StructuredTool.from_function(check_manufacturability),
        StructuredTool.from_function(submit_design),
        StructuredTool.from_function(search_docs),
        StructuredTool.from_function(search_parts),
        StructuredTool.from_function(preview_part),
        StructuredTool.from_function(read_journal),
        StructuredTool.from_function(lint_script),
        StructuredTool.from_function(list_skills),
        StructuredTool.from_function(list_skill_files),
        StructuredTool.from_function(read_skill),
        StructuredTool.from_function(update_skill),
        StructuredTool.from_function(init_skill),
        StructuredTool.from_function(package_skill),
        StructuredTool.from_function(run_skill_script),
    ]
