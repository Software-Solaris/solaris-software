# Solaris Software Development

# Guía de Instalación para Usuarios de Linux

Esta guía te ayudará a instalar las herramientas necesarias, configurar claves SSH, y trabajar con un repositorio en GitLab dentro de un contenedor Dev en Visual Studio Code.

---

## 1. Instalar Git
Ejecuta el siguiente comando en tu terminal para instalar Git:
```bash
sudo apt update
sudo apt install git -y
```

---

## 2. Instalar Visual Studio Code
Sigue estos pasos:
1. Descarga el paquete `.deb` de Visual Studio Code desde [la página oficial](https://code.visualstudio.com/).
2. Instálalo usando:
   ```bash
   sudo dpkg -i <nombre_del_fichero>.deb
   sudo apt-get install -f
   ```

---

## 3. Generar Claves SSH
1. Genera una clave SSH:
   ```bash
   ssh-keygen -t ed25519 -C "tu_email@example.com"
   ```
2. Todo enter en cualquier cosa que te pida.

---

## 4. Guardar la Clave SSH en GitLab
1. Copia tu clave pública:
   ```bash
   cat ~/.ssh/id_ed25519.pub
   ```
2. Ve al repo de solaris de [GitLab](https://gitlab.com/) y accede a: **Settings > SSH Keys**.
3. Pega la clave copiada y guárdala.

---

## 5. Verificar Conexión SSH con GitLab
Ejecuta:
```bash
ssh -T git@gitlab.com
```
---
 alfkjsdlñjfñsf
## 6. Clonar el Repositorio
1. Ve a tu proyecto en [GitLab](https://gitlab.com/) y copia el enlace SSH del repositorio.
2. En la terminal, navega a la carpeta donde quieres clonar el repositorio:
   ```bash
   cd <ruta_deseada>
   ```
3. Clona el repositorio:
   ```bash
   git clone git@gitlab.com:<usuario>/<nombre_repositorio>.git
   ```

---

## 7. Configuración del Proyecto
1. Ve a la carpeta `json` dentro del proyecto clonado:
   ```bash
   cd <nombre_repositorio>/json
   ```
2. Copia el contenido de Linux en el fichero `.devcontainer.json`.

---

## 8. Configurar Usuario de Git
Ejecuta los siguientes comandos para establecer tu nombre y correo:
```bash
sudo su
git config --global user.name "Tu Nombre"
git config --global user.email "tu_email@example.com"
```

---

## 9. Instalar la Extensión Dev Containers en Visual Studio Code
1. Abre [Visual Studio Code](https://code.visualstudio.com/).
2. Instala la extensión **Dev Containers** desde el marketplace.

---

## 10. Abrir el Proyecto en un Contenedor
1. Abre el proyecto en Visual Studio Code.
2. Selecciona **Reopen in Container** desde la paleta de comandos (`Ctrl+Shift+P`).
