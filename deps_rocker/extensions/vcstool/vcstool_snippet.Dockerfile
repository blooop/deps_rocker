RUN --mount=type=cache,target=/root/.cache/pip,id=pip-cache pip install vcstool --break-system-packages

ENV ROS_WORKSPACE_ROOT=@(workspace_root)
ENV ROS_REPOS_ROOT=@(repos_root)
ENV ROS_DEPENDENCIES_ROOT=@(dependencies_root)

RUN mkdir -p @(repos_root) @(dependencies_root)

# Import each discovered manifest into the canonical ROS workspace layout
@[for dep in depend_repos]@
COPY @(dep["dep"]) @(repos_root)/@(dep["dep"])
RUN mkdir -p @(dependencies_root)/@(dep["path"]) && \
    vcs import --recursive @(dependencies_root)/@(dep["path"]) < @(repos_root)/@(dep["dep"])
@[end for]@

RUN chmod -R a+rwX @(dependencies_root)
