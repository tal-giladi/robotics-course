"""robot_agent - module 19's small reference package: LLMs and task executives over a robot skill API.

sim_world      the simulated home: places, objects, fake detector and arm, navigation (robotlab.sim)
skills         the skill API: contracts, JSON schemas, validation, the logged gateway (19.02)
llm            provider-neutral messages + offline mock LLMs (19.03)
anthropic_llm  Claude adapter (Messages API tool use) (19.03)
agent_loop     the tool-calling loop with budgets and logging (19.03)
fsm            hierarchical state machines + the fetch task (19.04)
bt             the fetch task as a py_trees behavior tree (19.05)
"""
