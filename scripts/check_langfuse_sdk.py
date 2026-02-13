from langfuse.langchain import CallbackHandler
import inspect

print("CallbackHandler constructor signature:")
print(inspect.signature(CallbackHandler.__init__))
