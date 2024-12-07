WORKDIR /workspaces

#loops through all the *.repos files that were found and imports them with the same folder structure
@[for dep in depend_repos]@
COPY @dep["dep"] /dependencies/@dep["dep"]
RUN mkdir -p ./dependencies/@dep["path"] ; vcs import ./dependencies/@dep["path"] < /dependencies/@dep["dep"]
@[end for]@

RUN echo "source /usr/share/vcstool-completion/vcs.bash" >> $HOME/.bashrc