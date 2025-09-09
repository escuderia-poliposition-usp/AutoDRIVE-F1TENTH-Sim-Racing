# 1 BUILD DO SIMULADOR 

  ```bash
  docker build \
    --tag autodriveecosystem/autodrive_f1tenth_sim:latest \
    -f autodrive_simulator.Dockerfile \
    .
  ```

## 1.1- HABILITAR ACESSOS

```bash
xhost +local:root
```

## 1.2 GERAR CONTAINER (APLICAÇÃO)

```bash
docker run --name autodrive_f1tenth_sim --rm -it \
  --entrypoint /bin/bash \
  --network host --ipc host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY \
  --privileged --gpus all \
  autodriveecosystem/autodrive_f1tenth_sim:latest
```

## 1.3 ACESSAR O SIMULADOR

```bash
xhost +local:root
cd /home/autodrive_simulator
./AutoDRIVE\ Simulator.x86_64
```

# 2 F1TENTH AutoDRIVE — Fork da equipe (Dev Workspace + Compose)

Este fork adapta o repositório oficial **AutoDRIVE-F1TENTH-Sim-Racing** para um fluxo de desenvolvimento mais rápido:

- Workspace **colcon** em `autodrive_devkit/ws/src/` (múltiplos pacotes lado a lado).
- **Dockerfile DEV** sem copiar código (código entra via **bind-mount**).
- **Pins** de ferramentas Python (`pip`/`setuptools`/`wheel`/`packaging`) para o `--symlink-install` funcionar com `ament_python`.
- **compose.yaml** para subir o container e compilar com um comando.
- Pacotes Nav2 instalados na imagem (APT): `nav2_bringup`, `slam_toolbox`, `robot_state_publisher`, `joint_state_publisher`, `xacro`, etc.

> O README oficial foi movido para `docs/UPSTREAM_README.md`.

---

## Estrutura do repo (resumo)

├── README.md
├── compose.yaml
├── autodrive_devkit.Dockerfile # imagem DEV (não copia código)
├── autodrive_simulator/
├── autodrive_simulator.Dockerfile
└── autodrive_devkit/
└── ws/
└── src/
├── autodrive_f1tenth/ # ament_python (nós/bridge/launch/rviz)
└── f1tenth_description/ # ament_cmake (URDF/xacro/launch)

---


---

## Pré-requisitos

- Ubuntu 22.04 + Docker + Docker Compose v2.
- Se for usar GPU, instale e configure o **NVIDIA Container Toolkit**:

  ```bash
  distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

  curl -fsSL https://nvidia.github.io/libnvidia-container/ubuntu22.04/libnvidia-container.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#' \
    | sudo tee /etc/apt/sources.list.d/libnvidia-container.list

  sudo apt-get update
  sudo apt-get install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo systemctl restart docker

Verifique:

docker run --rm --gpus all nvidia/cuda:12.2.0-base-ubuntu22.04 nvidia-smi

Liberar X11 para GUIs (RViz, gedit, etc.):

xhost +local:root


## Como Rodar do Zero

# A) clone

```bash
git clone https://github.com/escuderia-poliposition-usp/AutoDRIVE-F1TENTH-Sim-Racing.git 
cd AutoDRIVE-F1TENTH-Sim-Racing
```

# B) habilitar X11 (para RViz/gedit etc.)

  ```bash
  xhost +local:root
  ```

# C) build da imagem e subir o container

  ```bash
    docker compose down
    docker compose build --no-cache devkit
    docker compose up -d
    docker compose ps    # deve mostrar autodrive_f1tenth_api: Up
  ```

# D) entrar no container

  ```bash
    docker compose exec devkit bash
  ```

# E) compilar o workspace

  ```bash
    source /opt/ros/humble/setup.bash
    cd /home/autodrive_devkit/ws
    colcon list
    colcon build --symlink-install
    source install/setup.bash
  ```

# F) smoke test (URDF)

  ```bash
    ros2 launch f1tenth_description bringup.launch.py gui:=true
  ```

## 2.1 Fluxo de desenvolvimento (rápido)

Edite arquivos em autodrive_devkit/ws/src/... no host.

No container:

  ```bash
    cd /home/autodrive_devkit/ws
    colcon build --symlink-install
    source install/setup.bash
  ```

### Relance seus launch e pronto (sem rebuildar a imagem). Se preferir, você pode ignorar o pacote Python no colcon e instalar em modo editável (efeito idêntico ao symlink):

  ```bash
    touch src/autodrive_f1tenth/COLCON_IGNORE
    pip3 install -e src/autodrive_f1tenth
    colcon build --symlink-install --packages-skip autodrive_f1tenth
  ```

## compose.yaml esperado (resumo)

  ```bash
    services:
      devkit:
        build:
          context: .
          dockerfile: autodrive_devkit.Dockerfile
        image: autodrive_f1tenth_api:dev
        container_name: autodrive_f1tenth_api
        network_mode: host
        ipc: host
        privileged: true
        environment:
          - NVIDIA_VISIBLE_DEVICES=all
          - NVIDIA_DRIVER_CAPABILITIES=compute,utility
          - DISPLAY=${DISPLAY}
          - XAUTHORITY=${XAUTHORITY:-}
        volumes:
          - /tmp/.X11-unix:/tmp/.X11-unix:rw
          - ./autodrive_devkit/ws:/home/autodrive_devkit/ws:rw
          - ./.colcon-cache:/home/autodrive_devkit/ws/.colcon-cache
        working_dir: /home/autodrive_devkit/ws
        restart: unless-stopped
        command: ["bash", "-lc", "tail -f /dev/null"]

  ```

## Diferenças principais vs. upstream

Layout: criamos um workspace real autodrive_devkit/ws/src/. O pacote autodrive_f1tenth foi movido para lá.

Novo pacote: adicionamos f1tenth_description (URDF/xacro/launch).

Dockerfile DEV: não copia código; instala Nav2 e utilitários; fixa versões de pip/setuptools/wheel/packaging (e zope-event<5.0) para viabilizar --symlink-install.

Compose: dev flow com bind-mount do ws, build rápido e shell interativo.

Troubleshooting
Erro --editable ou --uninstall no pacote Python (ament_python)
Já mitigado no Dockerfile. Garanta também um pyproject.toml no pacote:


# autodrive_devkit/ws/src/autodrive_f1tenth/pyproject.toml

  ```bash
    [build-system]
    requires = ["setuptools>=65,<66", "wheel<0.42"]
    build-backend = "setuptools.build_meta"
  ```

## Fallback rápido:

  ```bash
    touch src/autodrive_f1tenth/COLCON_IGNORE
    pip3 install -e src/autodrive_f1tenth
    colcon build --symlink-install --packages-skip autodrive_f1tenth
  ```

GUI (gedit/rviz) reclamando de dconf/dbus

A imagem já inclui dbus-x11. Lembre-se do xhost +local:root.

## Permissões (se usou sudo no host):

  ```bash
    sudo chown -R $USER:$USER autodrive_devkit/ws .colcon-cache
  ```

# 3 Acessar o ambiente e Listar tópicos

  ```bash
    docker exec -it autodrive_f1tenth_api bash
  ```

# 4 ATALHOS DE TERMINAL (aliases)

## Para evitar digitar comandos longos, crie aliases no seu shell.

## 4.1 Como habilitar

### Abra seu ~/.bashrc (ou ~/.zshrc se usar zsh):

  ```bash
    gedit ~/.bashrc
  ```

### Cole no final do arquivo:

    ```bash
    # --- F1TENTH aliases ---------------------------------------------------------

    # (Requer: estar dentro do diretório da repo com compose.yaml)
    alias dup_dev='docker compose up -d'
    alias dex_dev='docker compose exec devkit bash'

    # Build da imagem do simulador (Dockerfile: autodrive_simulator.Dockerfile)
    alias dup_sim="docker build \
      --tag autodriveecosystem/autodrive_f1tenth_sim:latest \
      -f autodrive_simulator.Dockerfile \
      ."

    # Rodar o container do simulador com X11 e GPU
    alias dex_sim='docker run --name autodrive_f1tenth_sim --rm -it \
      --entrypoint /bin/bash \
      --network host --ipc host \
      -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      -e DISPLAY=$DISPLAY -e XAUTHORITY=$XAUTHORITY \
      --privileged --gpus all \
      autodriveecosystem/autodrive_f1tenth_sim:latest'

    # ---------------------------------------------------------------------------
  ```

### Recarregue o shell 

  ```bash
    source ~/.bashrc
  ```

## Uso Rápido 

## Carregar Simulador 

  ```bash
  xhost +local:root        # faça uma vez por sessão gráfica
  dup_sim                  # build da imagem do simulador
  dex_sim                  # entra no container do simulador
  # dentro do container:
  cd /home/autodrive_simulator
  ./AutoDRIVE\ Simulator.x86_64
  ```

## Carregar Devkit (em outro terminal)

  ```bash
  cd ~/AutoDRIVE-F1TENTH-Sim-Racing
  dup_dev                  # sobe o devkit
  dex_dev                  # entra no devkit
  ```

## Licença e crédito

Este fork deriva de AutoDRIVE-F1TENTH-Sim-Racing (AutoDRIVE Ecosystem).
O README original pode ser encontrado em docs/UPSTREAM_README.md. Mantemos as licenças originais conforme package.xml e LICENSE.

