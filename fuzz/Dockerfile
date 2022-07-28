FROM ubuntu:20.04

#############
# ROOT USER #
#############

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
      afl++ \
      build-essential \
      cmake \
      g++-multilib \
      gcc-multilib \
      git \
      sudo \
      wget

ENV RUSTUP_HOME=/opt/rust
ENV CARGO_HOME=/opt/cargo

ENV PATH=/opt/cargo/bin:/opt/rust/bin:${PATH}

RUN wget "https://static.rust-lang.org/rustup/dist/x86_64-unknown-linux-gnu/rustup-init"; \
    chmod +x rustup-init; \
    ./rustup-init -y --no-modify-path --default-toolchain stable; \
    rustup toolchain install nightly-2022-07-19; \
    rm rustup-init; \
    chmod -R a+w $RUSTUP_HOME $CARGO_HOME; \
    rustup --version; \
    cargo --version; \
    rustc --version

RUN cargo install cargo-fuzz

RUN useradd -u 1001 -ms /bin/bash -G sudo,staff jenkins
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN chown -R jenkins /opt/cargo

################
# JENKINS USER #
################

USER jenkins

RUN mkdir -p $HOME/.ssh && chmod go-rwx $HOME/.ssh
RUN ssh-keyscan github.com >> $HOME/.ssh/known_hosts

RUN git config --global user.email 'swiftnav-svc-jenkins@swift-nav.com' \
 && git config --global user.name 'Swift Jenkins'