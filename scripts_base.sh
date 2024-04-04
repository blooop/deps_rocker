
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -sc) robotpkg
EOF

curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -