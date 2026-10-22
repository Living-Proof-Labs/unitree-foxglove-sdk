FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    UV_COMPILE_BYTECODE=1 \
    UV_LINK_MODE=copy \
    UV_PYTHON_INSTALL_DIR=/opt/uv-python \
    PATH="/app/.venv/bin:${PATH}" \
    UNITREE_MUJOCO_ROOT=/app/third_party/unitree_mujoco \
    UNITREE_RL_GYM_ROOT=/app/third_party/unitree_rl_gym \
    SDL_VIDEODRIVER=dummy \
    SDL_AUDIODRIVER=dummy

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        libegl1 \
        libgl1 \
        libgles2 \
        libglfw3 \
        libglib2.0-0 \
        libopengl0 \
        xvfb \
    && rm -rf /var/lib/apt/lists/*

COPY --from=ghcr.io/astral-sh/uv:0.12.19 /uv /uvx /usr/local/bin/

WORKDIR /app

COPY pyproject.toml uv.lock README.md ./
COPY third_party/unitree_sdk2_python third_party/unitree_sdk2_python
COPY src src
COPY tests tests

RUN uv python install 3.10 \
    && uv sync --frozen --all-groups \
    && chmod -R a+rX /opt/uv-python /app

COPY third_party/unitree_mujoco third_party/unitree_mujoco
# Only the pretrained G1 policy and its deploy config; the training code needs Isaac Gym.
COPY third_party/unitree_rl_gym/LICENSE third_party/unitree_rl_gym/LICENSE
COPY third_party/unitree_rl_gym/deploy/pre_train/g1/motion.pt \
     third_party/unitree_rl_gym/deploy/pre_train/g1/motion.pt
COPY third_party/unitree_rl_gym/deploy/deploy_real/configs/g1.yaml \
     third_party/unitree_rl_gym/deploy/deploy_real/configs/g1.yaml
# URDF and meshes the bridge serves to the Foxglove 3D panel.
COPY third_party/unitree_rl_gym/resources/robots/g1_description/g1_29dof.urdf \
     third_party/unitree_rl_gym/resources/robots/g1_description/g1_29dof.urdf
COPY third_party/unitree_rl_gym/resources/robots/g1_description/meshes \
     third_party/unitree_rl_gym/resources/robots/g1_description/meshes

CMD ["unitree-sim"]
