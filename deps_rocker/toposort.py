from graphlib import TopologicalSorter


graph = {"D": {"B", "C"}, "C": {"A"}, "B": {"A"}}
ts = TopologicalSorter(graph)
print(list(ts.static_order()))
