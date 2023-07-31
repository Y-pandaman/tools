#!/usr/bin/env bash

set -ex

# install python3
apt-get update &&
    apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-dev &&
    apt-get clean &&
    rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*
python3 -m pip install -U pip

# install extra tools
apt-get update &&
    apt-get install -y --no-install-recommends \
        gawk \
        tmux \
        zsh \
        vim \
        htop \
        iotop \
        iftop \
        gdb &&
    apt-get clean &&
    rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*

# install zsh plugins
sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended &&
    git clone https://github.com/zsh-users/zsh-autosuggestions.git $HOME/.oh-my-zsh/custom/plugins/zsh-autosuggestions &&
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git $HOME/.oh-my-zsh/custom/plugins/zsh-syntax-highlighting &&
    git clone https://github.com/zsh-users/zsh-completions.git $HOME/.oh-my-zsh/custom/plugins/zsh-completions

# set zsh as default shell
chsh -s /bin/zsh root

# install tmux plugin manager
mkdir -p $HOME/.tmux/plugins && git clone https://github.com/tmux-plugins/tpm $HOME/.tmux/plugins/tpm

# install clangd binary by hand
# ARG CLANGD_VERSION=11.0.0
# RUN cd /tmp && \
#   wget https://github.com/clangd/clangd/releases/download/${CLANGD_VERSION}/clangd-linux-${CLANGD_VERSION}.zip && \
#   unzip clangd-linux-${CLANGD_VERSION}.zip && \
#   rsync -avP clangd_${CLANGD_VERSION}/lib/ /usr/local/lib/ && \
#   rsync -avP clangd_${CLANGD_VERSION}/bin/ /usr/local/bin/ && \
#   cd / && rm -rf /tmp/*

# install clangd via apt
apt-get update &&
    apt-get install -y --no-install-recommends clang-format isort flake8 &&
    apt-get install -y --no-install-recommends black || true &&
    apt-get install -y --no-install-recommends yapf || apt-get install -y --no-install-recommends yapf3 &&
    apt-get install -y --no-install-recommends clangd || apt-get install -y --no-install-recommends clangd-10 &&
    apt-get clean &&
    rm -rf /tmp/* /var/lib/apt/lists/* /var/tmp/*
