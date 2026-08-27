FROM osrf/ros:lyrical-desktop-full
ARG USERNAME=rafacho
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL=/bin/zsh

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************



############################################################
# General development tools
############################################################

RUN apt-get update && \
    apt-get install -y \
        build-essential \
        cmake \
        pkg-config \
        git \
        curl \
        wget \
        unzip \
        python3-pip \
        python3-colcon-common-extensions \
        xauth \
        dbus-x11 \
        x11-apps \
        zsh \
        fonts-powerline \
        chroma \
        libeigen3-dev \
        libgdal-dev \
        libgeos-dev \
        libcgal-dev \
        libtbb-dev \
        libomp-dev \
        libtinyxml2-dev \
        && rm -rf /var/lib/apt/lists/*

############################################################
# ROS packages
############################################################

RUN apt-get update && \
    apt-get install -y \
        ros-lyrical-ros-gz \
        ros-lyrical-ros-gz-bridge \
        ros-lyrical-ros-gz-sim \
        ros-lyrical-gz-ros2-control \
        ros-lyrical-ros2-control \
        ros-lyrical-ros2-controllers \
        ros-lyrical-controller-manager \
        ros-lyrical-diff-drive-controller \
        ros-lyrical-joint-state-broadcaster \
        ros-lyrical-robot-state-publisher \
        ros-lyrical-xacro \
        ros-lyrical-teleop-twist-keyboard \
        ros-lyrical-robot-localization \
        ros-lyrical-topic-tools \
        ros-lyrical-rqt-tf-tree \
        ros-lyrical-tf-transformations \
        ros-lyrical-joint-state-publisher-gui \
        ros-lyrical-plotjuggler \
        python3-pil \
        && rm -rf /var/lib/apt/lists/*

# plotjuggler note (2026-08-27): switched from ros-lyrical-plotjuggler-ros to
# the standalone ros-lyrical-plotjuggler. The -ros package (which adds the ROS2
# topic-streaming plugin) is not in the Lyrical repo as of the 2026-08-17
# osrf/ros:lyrical-desktop-full base-image refresh -- that refresh is also why
# a rebuild that worked earlier now fails here with exit 100 / "Unable to
# locate package ros-lyrical-plotjuggler-ros". plotjuggler isn't referenced
# anywhere in this workspace (dev-environment extra). Re-add
# ros-lyrical-plotjuggler-ros once it's released, if ROS2 streaming is needed.

# ROS version switch note (2026-08-06): the nav2-* packages (nav2-bringup,
# nav2-bt-navigator, nav2-controller, nav2-planner, nav2-behaviors,
# nav2-map-server, nav2-amcl, nav2-lifecycle-manager, nav2-waypoint-follower,
# nav2-velocity-smoother, nav2-regulated-pure-pursuit-controller) that used to
# be installed here for Jazzy are dropped: not yet released for Lyrical Luth
# as of this switch, and confirmed unused anywhere in this workspace's own
# packages (they were dev-environment extras, not a real dependency). Re-add
# once ros-lyrical-nav2-* packages exist, if Nav2 is actually needed.

# ROS2 port note (2026-08-04): pyquaternion (used by pandora_control's
# wheel_contacts.py/static_stability.py) has no ROS/apt package -- installed
# via pip. numpy, matplotlib, rviz2, message_filters, tf2_geometry_msgs and
# ros_gz_interfaces are already covered by ros-lyrical-desktop-full / the
# ros-gz packages above, so they don't need a separate line here.
RUN pip install --break-system-packages --no-cache-dir pyquaternion


############################################################
# Configure user
############################################################

RUN chsh -s $(which zsh) ${USERNAME}

USER ${USERNAME}

WORKDIR /home/${USERNAME}

############################################################
# Oh My Zsh
############################################################

RUN sh -c "$(curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh)" "" --unattended

############################################################
# ZSH plugins
############################################################

RUN for plugin in \
        zsh-users/zsh-autosuggestions \
        zsh-users/zsh-syntax-highlighting \
        zpm-zsh/colorize; do \
        git clone https://github.com/$plugin \
        /home/${USERNAME}/.oh-my-zsh/custom/plugins/$(basename $plugin); \
    done

############################################################
# Configure .zshrc
############################################################

RUN sed -i 's/ZSH_THEME="robbyrussell"/ZSH_THEME="agnoster"/' /home/$USERNAME/.zshrc && \
    sed -i 's/plugins=(git)/plugins=(git zsh-autosuggestions zsh-syntax-highlighting colorize)/' /home/$USERNAME/.zshrc && \
    echo "source /opt/ros/lyrical/setup.zsh" >> /home/$USERNAME/.zshrc && \
    echo "[ -f /home/$USERNAME/ros_lyrical_docker_ws/install/setup.zsh ] && source /home/$USERNAME/ros_lyrical_docker_ws/install/setup.zsh" >> /home/$USERNAME/.zshrc && \
    { \
        echo ""; \
        echo "# History: bind-mounted to the host (see docker-compose.yml) so it"; \
        echo "# survives container recreation; shared/appended incrementally so"; \
        echo "# concurrent \`docker exec\` shells don't clobber each other's history."; \
        echo "HISTFILE=\$HOME/.zsh_history"; \
        echo "HISTSIZE=50000"; \
        echo "SAVEHIST=50000"; \
        echo "setopt APPEND_HISTORY"; \
        echo "setopt INC_APPEND_HISTORY"; \
        echo "setopt SHARE_HISTORY"; \
    } >> /home/$USERNAME/.zshrc

############################################################
# Workspace
############################################################

WORKDIR /home/${USERNAME}/ros_lyrical_docker_ws

# COPY --chown=${USERNAME}:${USERNAME} src ./src

############################################################
# Build workspace
############################################################

# Workspace is mounted at runtime by VS Code, so build it in postCreateCommand instead.
# If you want to build at image build time, copy the workspace into the image first.
# RUN /bin/bash -c "\
# source /opt/ros/lyrical/setup.bash && \
# colcon build --symlink-install"

############################################################
# Entry point
############################################################

# ENTRYPOINT ["/bin/zsh", "-c", "source /opt/ros/lyrical/setup.zsh && if [ -f /home/rafacho/ros_lyrical_docker_ws/install/setup.zsh ]; then source /home/rafacho/ros_lyrical_docker_ws/install/setup.zsh; fi && exec \"$@\"","--"]

ENTRYPOINT ["/bin/zsh", "-c", "source /opt/ros/lyrical/setup.zsh; [ -f /home/$USERNAME/ros_lyrical_docker_ws/install/setup.zsh ] && source /home/$USERNAME/ros_lyrical_docker_ws/install/setup.zsh; exec \"$@\"", "--"]

CMD ["zsh"]




# [Optional] Set the default user. Omit if you want to keep the default as root.
# USER $USERNAME
# CMD ["/bin/zsh"]