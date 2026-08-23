# GPU Traffic Simulator

Simulateur de trafic routier 3D conçu pour apprendre progressivement la programmation GPU et l'IA à travers un projet concret.

## Objectifs

* C++ moderne pour le moteur de simulation
* CUDA C++ pour les calculs parallèles
* raylib pour la visualisation 3D
* Dear ImGui pour l'interface et les métriques
* NVIDIA Nsight pour le profiling
* Python + PyTorch plus tard pour l'IA

Le projet privilégie l'apprentissage pratique : les mécanismes importants sont d'abord construits et compris avant d'être optimisés.

---

## État actuel

Le premier jalon valide la chaîne de développement complète :

* compilation C++ avec MSVC ;
* compilation CUDA avec `nvcc` ;
* build avec CMake ;
* détection du GPU NVIDIA ;
* transfert de données CPU → GPU → CPU ;
* exécution d'un premier kernel CUDA ;
* scène 3D minimale avec raylib.

Le kernel actuel traite **1024 éléments** avec **256 threads par block**, soit **4 blocks**.

---

# Installation sous Windows 11

Le setup ci-dessous correspond à l'environnement utilisé pour développer le projet : **Windows 11 + GPU NVIDIA + Visual Studio 2022 + CUDA Toolkit 13.3**.

## 1. Vérifier le GPU NVIDIA

Dans PowerShell :

```powershell
nvidia-smi
```

Le GPU doit apparaître correctement.

> La version CUDA affichée par `nvidia-smi` indique ce que le **driver NVIDIA supporte**. Elle ne signifie pas forcément que le **CUDA Toolkit** et `nvcc` sont installés.

---

## 2. Installer Git

Vérifier d'abord :

```powershell
git --version
```

Si Git est installé, aucune autre action n'est nécessaire.

---

## 3. Installer Visual Studio Community 2022

CUDA sous Windows a besoin d'un compilateur C++ hôte. Ici, nous utilisons **MSVC**, fourni par Visual Studio 2022.

Installation :

```powershell
winget install Microsoft.VisualStudio.2022.Community
```

Ensuite ouvrir **Visual Studio Installer** :

```text
Visual Studio Community 2022
→ Modifier
→ Charges de travail
→ Développement Desktop en C++
```

Vérifier notamment la présence de :

* MSVC v143 ;
* outils de build C++ x64/x86 ;
* Windows SDK.

### Visual Studio ou VS Code ?

Les deux peuvent être utilisés pour coder.

Dans notre setup, **VS Code sert d'éditeur**, tandis que **Visual Studio fournit MSVC** pour la compilation :

```text
VS Code
   ↓
CMake
   ↓
MSVC + nvcc
   ↓
Executable
   ↓
GPU NVIDIA
```

---

## 4. Installer CMake

```powershell
winget install Kitware.CMake
```

Fermer puis rouvrir le terminal ou VS Code, puis vérifier :

```powershell
cmake --version
```

---

## 5. Installer NVIDIA CUDA Toolkit

Installer le **CUDA Toolkit** depuis NVIDIA.

Le Toolkit fournit notamment :

* `nvcc` ;
* le CUDA Runtime ;
* les headers et bibliothèques CUDA ;
* les outils de développement NVIDIA ;
* l'intégration Visual Studio.

Le projet a été validé avec :

```text
CUDA Toolkit 13.3
nvcc 13.3
Visual Studio Community 2022
```

Après l'installation, fermer puis rouvrir les terminaux et VS Code.

Vérifier :

```powershell
nvcc --version
echo $env:CUDA_PATH
```

Exemple :

```text
Cuda compilation tools, release 13.3
C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.3
```

---

## 6. Vérification complète

Avant de compiler le projet :

```powershell
nvidia-smi
nvcc --version
cmake --version
git --version
echo $env:CUDA_PATH
```

L'environnement doit ressembler à ceci :

```text
GPU NVIDIA          OK
Driver NVIDIA       OK
Git                 OK
CMake               OK
Visual Studio/MSVC  OK
CUDA Toolkit        OK
nvcc                OK
CUDA_PATH           OK
```

---

# VS Code

## Extensions utiles

* C/C++
* CMake Tools
* NVIDIA Nsight Visual Studio Code Edition, plus tard si nécessaire

Lors de la première configuration, CMake Tools peut demander :

```text
Select a Kit
```

Choisir :

```text
Visual Studio Community 2022 Release - amd64
```

Le kit `amd64` correspond à la compilation native 64 bits que nous utilisons.

---

# Récupérer le projet

```powershell
cd C:\dev
git clone <URL_DU_REPO>
cd gpu-traffic-simulator
```

---

# Structure actuelle

```text
gpu-traffic-simulator/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── assets/
├── include/
└── src/
    ├── main.cpp
    ├── cuda/
    │   ├── cuda_test.cu
    │   └── cuda_test.cuh
    ├── rendering/
    ├── simulation/
    └── ui/
```

Les dossiers `rendering`, `simulation` et `ui` sont volontairement encore simples. Ils seront remplis progressivement.

---

# Compiler le projet

## 1. Configurer CMake

Depuis la racine du dépôt :

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
```

Cette commande :

* utilise le dossier courant comme source ;
* génère les fichiers de build dans `build/` ;
* utilise Visual Studio 2022 / MSVC ;
* compile en 64 bits.

Lors du premier lancement, CMake télécharge automatiquement **raylib** grâce à `FetchContent`.

Une configuration correcte doit notamment détecter :

```text
MSVC
NVIDIA CUDA Compiler
CUDA Toolkit
```

## 2. Compiler

```powershell
cmake --build build --config Debug
```

Le projet combine alors :

```text
main.cpp      → MSVC
cuda_test.cu  → nvcc
                    ↓
       gpu_traffic_simulator.exe
```

## 3. Exécuter

```powershell
.\build\Debug\gpu_traffic_simulator.exe
```

Le terminal devrait afficher quelque chose comme :

```text
GPU Traffic Simulator
=====================

=== CUDA test ===
CUDA device: NVIDIA GeForce RTX 5070
Elements: 1024
Threads per block: 256
Blocks: 4
Result: SUCCESS
```

Une fenêtre raylib s'ouvre ensuite avec une grille 3D et un cube de test.

Appuyer sur `ESC` pour quitter.

---

# Ce que teste le premier kernel CUDA

Le programme valide déjà le chemin fondamental utilisé plus tard par le simulateur :

```text
RAM CPU
   │
   │ cudaMalloc
   ▼
VRAM GPU
   │
   │ cudaMemcpy Host → Device
   ▼
Données GPU
   │
   │ kernel<<<blocks, threads>>>
   ▼
Calcul parallèle
   │
   │ cudaMemcpy Device → Host
   ▼
RAM CPU
   │
   └── vérification du résultat
```

Aujourd'hui, chaque thread ajoute simplement `1` à un élément du tableau.

Plus tard, le même principe sera utilisé pour mettre à jour en parallèle les états des véhicules.

---

# Dépannage

## `nvcc` n'est pas reconnu

Si :

```powershell
nvcc --version
```

retourne une erreur, vérifier :

```powershell
echo $env:CUDA_PATH
Get-ChildItem "C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA"
```

Puis tester directement :

```powershell
& "$env:CUDA_PATH\bin\nvcc.exe" --version
```

Si cette commande fonctionne, le Toolkit est installé et le problème vient probablement du `PATH`.

Fermer complètement VS Code et les terminaux puis les relancer.

## `cmake` n'est pas reconnu

Après :

```powershell
winget install Kitware.CMake
```

fermer puis rouvrir le terminal avant de tester :

```powershell
cmake --version
```

## Visual Studio est installé mais C++ ne compile pas

Ouvrir **Visual Studio Installer** puis vérifier :

```text
Modifier
→ Charges de travail
→ Développement Desktop en C++
```

MSVC v143 et le Windows SDK doivent être présents.

## `cl.exe` n'est pas reconnu dans PowerShell

Ce n'est pas forcément un problème. `cl.exe` n'est pas toujours présent dans le `PATH` d'un PowerShell classique.

Le test le plus important est que CMake arrive à configurer le projet avec :

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
```

## Raylib n'est pas installé manuellement

C'est normal. Le projet utilise `FetchContent` pour récupérer raylib automatiquement lors de la première configuration CMake.
