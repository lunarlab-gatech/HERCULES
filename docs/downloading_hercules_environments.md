# Downloading the HERCULES Environments from Fab

The photorealistic environments used in the HERCULES paper, website, and dataset
(desert, forest, and city) are built from asset packs downloaded from Epic's
[Fab](https://www.fab.com) marketplace. Epic's licensing does not allow us to
redistribute the raw assets in this repository, but **all three packs are
free** — each user simply needs to claim them under their own Epic Games
account and download them. This document walks through the exact steps.

> **Coming soon:** we are also preparing pre-packaged (cooked) simulator
> binaries of these environments, which the Fab license does permit us to
> distribute. Once released, you will only need the steps below if you want to
> open or modify the environments in the Unreal Editor. The packaged builds
> released so far are listed in [Packaged Environments](packaged_environments.md).

## The three environments

| HERCULES environment | Fab listing | Seller | Price |
|---|---|---|---|
| Desert / Australian outback | [Rural Australia](https://www.fab.com/listings/1c1467ce-a2f5-4be1-8988-9069f90a8571) | Andrew Svanberg Hamilton | Free |
| City | [City Sample](https://www.fab.com/listings/4898e707-7855-404b-af0e-a505ee690e68) | Epic Games | Free |
| Forest | [Chestnuts Pack (Aesculus hippocastanum)](https://www.fab.com/listings/d35f1b79-cc72-485c-947c-fbe524273254) | (see listing) | Free |

## Prerequisites

- An **Epic Games account** (free): https://www.epicgames.com
- The **Epic Games Launcher** installed on a **Windows or macOS** machine.
  There is no launcher for Linux — if your workstations are Linux-based (ours
  are, see reference machine below), download on any Windows/Mac machine and
  copy the files over (step 5). HERCULES targets **UE 5.8**, so the in-editor
  Fab plugin (UE 5.3+) is an alternative to the launcher where it is available.
- Disk space: the City Sample is by far the largest item (tens of GB
  downloaded and expanded). We recommend having **≥ 200 GB free** before
  starting.

## Steps

### 1. Claim the listings into your Fab library

While signed in to fab.com in a browser, open each of the three listing links
above and click **"Add to My Library"** (free listings). This is a one-time,
per-account step and unfortunately cannot be scripted — Epic gates downloads
on per-account entitlements.

### 2. Download via the Epic Games Launcher

In the Epic Games Launcher: **Unreal Engine → Library → Fab Library** (make
sure you are logged in with the same account). All three items should appear.

- **Rural Australia** and the **Chestnuts Pack** are *asset packs*: click
  **"Add to Project"** and select the target UE project.
- **City Sample** is a *complete project*: click **"Create Project"** and
  choose a location.

### 3. Engine-version note (important)

HERCULES targets **Unreal Engine 5.8**. If the launcher does not offer
"5.8" in the version dropdown for a pack (some packs list only older or newer
engine versions), use the standard workaround:

1. Download the pack for the **closest supported version** into a blank
   project (create a blank project with that engine version if needed).
2. Copy the pack's folder out of that project's `Content/` directory into the
   HERCULES project's `Content/` directory (see next step). Plain asset packs
   like these migrate across nearby engine versions without issues; the
   editor will recompile shaders on first load.

### 4. Where the files must end up

The HERCULES maps reference the packs at these exact locations under the UE
project's `Content/` folder — the folder names must match:

```
<Project>/Content/RuralAustralia/       <- Rural Australia
<Project>/Content/Chestnuts_Pack/       <- Chestnuts Pack (Aesculus hippocastanum)
<Project>/Content/CitySampleBuildings/  <- from City Sample
<Project>/Content/CitySampleCrowd/      <- from City Sample
<Project>/Content/CitySampleVehicles/   <- from City Sample
```

For the City Sample, copy the `CitySample*` folders from the downloaded
sample project's `Content/` directory into the HERCULES project.

If a map is opened before the packs are in place, the editor will show
missing-asset errors and empty terrain — re-check the folder names above.

### 5. Getting the content onto a Linux workstation

If you downloaded on a Windows/Mac machine, copy the relevant `Content/`
subfolders to the Linux machine with `scp`/`rsync` (or a USB drive). Copy
while the editor is closed. Asset files (`.uasset`, `.umap`) are
platform-independent — no conversion is needed.

## Reference machine

For hardware planning: all HERCULES development, dataset collection, and the
published sequences were produced on a single laptop with the following
specifications. Anything comparable or better will run the environments; the
City Sample in particular benefits from GPU VRAM and RAM headroom.

| Component | Spec |
|---|---|
| Machine | Lenovo Legion Pro 7 16IRX8H (model 21FB) |
| OS | Ubuntu 22.04.5 LTS (kernel 6.8) |
| CPU | Intel Core i9-13980HX (24 cores / 32 threads) |
| RAM | 96 GB |
| GPU | NVIDIA GeForce RTX 4090 Laptop GPU, 16 GB VRAM (driver 550.144) |
| Unreal Engine | 5.2.1, built from source on Linux (the repository now targets UE 5.8) |
| Storage | NVMe SSD; environments and datasets kept on external SSDs |

Note that UE on Linux uses the Vulkan RHI; on Windows the same projects run
under DX12. Both work with HERCULES.

## Questions

If a pack fails to download, a map opens with missing assets, or a listing
appears unavailable in your region, please open an issue on the HERCULES
GitHub repository or contact us by email.
