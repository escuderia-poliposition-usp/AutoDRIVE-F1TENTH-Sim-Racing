# F1TENTH AutoDRIVE — Fork da equipe (Dev Workspace + Compose)

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

## Pré-requisitos

- Ubuntu + Docker (e, se for usar GPU, NVIDIA Container Toolkit configurado).
- Display X11 liberado para GUIs em container:

  ```bash
  xhost +local:root
  ```

## Como Rodar do Zero

# 1) clone

```bash
git clone <URL-DO-FORK> AutoDRIVE-F1TENTH-Sim-Racing
cd AutoDRIVE-F1TENTH-Sim-Racing
```

# 2) habilitar X11 (para RViz/gedit etc.)

  ```bash
  xhost +local:root
  ```

# 3) build da imagem e subir o container

  ```bash
    docker compose down
    docker compose build --no-cache devkit
    docker compose up -d
    docker compose ps    # deve mostrar autodrive_f1tenth_api: Up
  ```

# 4) entrar no container

  ```bash
    docker compose exec devkit bash
  ```

# 5) compilar o workspace

  ```bash
    source /opt/ros/humble/setup.bash
    cd /home/autodrive_devkit/ws
    colcon list
    colcon build --symlink-install
    source install/setup.bash
  ```

# 6) smoke test (URDF)

  ```bash
    ros2 launch f1tenth_description bringup.launch.py gui:=true
  ```

## Fluxo de desenvolvimento (rápido)

Edite arquivos em autodrive_devkit/ws/src/... no host.

No container:

  ```bash
    cd /home/autodrive_devkit/ws
    colcon build --symlink-install
    source install/setup.bash
  ```

Relance seus launch e pronto (sem rebuildar a imagem).

Se preferir, você pode ignorar o pacote Python no colcon e instalar em modo editável (efeito idêntico ao symlink):

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
        gpus: all
        environment:
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

## Licença e crédito

Este fork deriva de AutoDRIVE-F1TENTH-Sim-Racing (AutoDRIVE Ecosystem).
O README original pode ser encontrado em docs/UPSTREAM_README.md. Mantemos as licenças originais conforme package.xml e LICENSE.

