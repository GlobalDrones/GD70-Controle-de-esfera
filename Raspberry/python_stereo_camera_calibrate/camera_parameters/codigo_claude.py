import cv2
import numpy as np
import os
import sys

# ===========================================================================
# 1. LEITURA DOS ARQUIVOS .dat DO REPO TEMUGEB
# Formato:
#   chave:
#   v1 v2 v3
#   v4 v5 v6
#   ...
# ===========================================================================

def load_dat(filepath):
    """
    Le um arquivo .dat do repo TemugeB.
    Retorna dicionario com chave -> np.array
    """
    result = {}
    current_key = None
    current_rows = []

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # Linha de chave termina com ':'
            if line.endswith(':'):
                # Salvar bloco anterior
                if current_key is not None and current_rows:
                    result[current_key] = np.array(current_rows)
                current_key = line[:-1]  # remove o ':'
                current_rows = []
            else:
                # Linha de dados
                try:
                    row = list(map(float, line.split()))
                    current_rows.append(row)
                except ValueError:
                    pass

    # Salvar ultimo bloco
    if current_key is not None and current_rows:
        result[current_key] = np.array(current_rows)

    return result


params_dir = os.path.join(os.path.dirname(__file__), '')  # mesma pasta do script

# Tenta encontrar a pasta camera_parameters
for candidate in [
    'camera_parameters',
    os.path.join('python_stereo_camera_calibrate', 'camera_parameters'),
    os.path.join(os.path.expanduser('~'), 'python_stereo_camera_calibrate', 'camera_parameters'),
]:
    if os.path.isdir(candidate):
        params_dir = candidate
        break
else:
    params_dir = input("Caminho da pasta camera_parameters: ").strip()

print(f"[INFO] Usando pasta: {params_dir}")

cam0_intr = os.path.join(params_dir, 'camera0_intrinsics.dat')
cam1_intr = os.path.join(params_dir, 'camera1_intrinsics.dat')
cam0_rt   = os.path.join(params_dir, 'camera0_rot_trans.dat')
cam1_rt   = os.path.join(params_dir, 'camera1_rot_trans.dat')

for p in [cam0_intr, cam1_intr, cam0_rt, cam1_rt]:
    if not os.path.exists(p):
        print(f"[ERRO] Arquivo nao encontrado: {p}")
        sys.exit(1)

# Carregar
d0 = load_dat(cam0_intr)
d1 = load_dat(cam1_intr)
r0 = load_dat(cam0_rt)
r1 = load_dat(cam1_rt)

cmtx0 = d0['intrinsic']        # 3x3
dist0 = d0['distortion']       # 1x5 ou 5x1
cmtx1 = d1['intrinsic']
dist1 = d1['distortion']

R0 = r0['R']                   # 3x3
T0 = r0['T'].flatten()         # (3,)
R1 = r1['R']
T1 = r1['T'].flatten()
for x in range(0,len(T1)):
    T1[x]=T1[x]/100

# Garantir shape correto
dist0 = dist0.flatten().reshape(1, -1)
dist1 = dist1.flatten().reshape(1, -1)

print("\n[OK] Parametros carregados")
print(f"  cmtx0:\n{cmtx0}")
print(f"  dist0: {dist0}")
print(f"  cmtx1:\n{cmtx1}")
print(f"  dist1: {dist1}")
print(f"  R0:\n{R0}")
print(f"  T0: {T0}")
print(f"  R1:\n{R1}")
print(f"  T1: {T1}")

# ===========================================================================
# 2. R e T RELATIVO (cam0 -> cam1)
#
# O repo define: P_cam = R @ P_world + T
#   P_cam0 = R0 @ P_world + T0
#   P_cam1 = R1 @ P_world + T1
#
# Isolando P_world em cam0:
#   P_world = R0^T @ (P_cam0 - T0)
#
# Substituindo em cam1:
#   P_cam1 = R1 @ R0^T @ P_cam0 + (T1 - R1 @ R0^T @ T0)
#
# Portanto:
#   R_rel = R1 @ R0^T
#   T_rel = T1 - R_rel @ T0
# ===========================================================================

R_rel = R1 @ R0.T
T_rel = (T1 - R_rel @ T0).reshape(3, 1)

baseline_m = np.linalg.norm(T_rel)
print(f"\n[INFO] R_rel:\n{R_rel}")
print(f"[INFO] T_rel: {T_rel.flatten()}")
print(f"[INFO] Baseline: {baseline_m*100:.2f} cm")

if baseline_m > 2.0:
    print("[AVISO] Baseline maior que 2 metros - provavel erro na leitura dos .dat")
    print("        Verifique os arquivos camera1_rot_trans.dat")
    sys.exit(1)

# ===========================================================================
# 3. CARREGAR IMAGENS
# ===========================================================================

frames_dir = 'frames_pair'
left_path  = None
right_path = None

# Busca automatica por pares com mesmo indice
if os.path.exists(frames_dir):
    files     = sorted(os.listdir(frames_dir))
    cam0_list = [f for f in files if f.startswith('camera0') and f.endswith('.png')]
    cam1_list = set(f for f in files if f.startswith('camera1') and f.endswith('.png'))

    for f0 in cam0_list:
        idx   = f0.split('_')[-1]          # ex: "0.png"
        match = f'camera1_{idx}'
        if match in cam1_list:
            left_path  = os.path.join(frames_dir, f0)
            right_path = os.path.join(frames_dir, match)
            print(f"\n[INFO] Par encontrado: {f0} <-> {match}")
            break

if left_path is None:
    left_path  = input("Caminho da imagem esquerda (camera0): ").strip()
    right_path = input("Caminho da imagem direita  (camera1): ").strip()

img_left  = cv2.imread(left_path)
img_right = cv2.imread(right_path)

if img_left is None or img_right is None:
    print(f"[ERRO] Nao foi possivel carregar as imagens")
    sys.exit(1)

img_size = (img_left.shape[1], img_left.shape[0])
print(f"[OK] Imagens carregadas: {img_size}")

if (img_right.shape[1], img_right.shape[0]) != img_size:
    img_right = cv2.resize(img_right, img_size)

# ===========================================================================
# 4. RETIFICACAO ESTEREO
# ===========================================================================

R1r, R2r, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
    cmtx0, dist0,
    cmtx1, dist1,
    img_size,
    R_rel, T_rel,
    alpha=0  # 0 = so pixels validos, sem bordas pretas
)

map1x, map1y = cv2.initUndistortRectifyMap(cmtx0, dist0, R1r, P1, img_size, cv2.CV_32FC1)
map2x, map2y = cv2.initUndistortRectifyMap(cmtx1, dist1, R2r, P2, img_size, cv2.CV_32FC1)

rect_left  = cv2.remap(img_left,  map1x, map1y, cv2.INTER_LINEAR)
rect_right = cv2.remap(img_right, map2x, map2y, cv2.INTER_LINEAR)

cv2.imwrite("rect_left.png",  rect_left)
cv2.imwrite("rect_right.png", rect_right)

# ===========================================================================
# 5. DIAGNOSTICO: linhas epipolares
# Objetos correspondentes devem estar na MESMA linha horizontal.
# ===========================================================================

def draw_epilines(left, right, step=40):
    h = left.shape[0]
    out = np.hstack([left.copy(), right.copy()])
    for y in range(step, h, step):
        color = [int(c) for c in np.random.randint(80, 255, 3)]
        cv2.line(out, (0, y), (out.shape[1], y), color, 1)
    return out

epi = draw_epilines(rect_left, rect_right)
cv2.imwrite("epipolar_check.png", epi)
cv2.imshow("Epipolar Check", epi)
print("\n[INFO] Salvo: epipolar_check.png")
print("       Se a retificacao estiver certa, objetos correspondentes")
print("       aparecem na MESMA linha horizontal nas duas imagens.")
cv2.waitKey(0)

# ===========================================================================
# 6. MAPA DE DISPARIDADE (SGBM + filtro WLS)
# ===========================================================================

grayL = cv2.cvtColor(rect_left,  cv2.COLOR_BGR2GRAY)
grayR = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

block    = 5
num_disp = 16 * 10  # 160 disparidades aumente para objetos muito proximos

left_matcher = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=num_disp,
    blockSize=block,
    P1=8  * 3 * block**2,
    P2=32 * 3 * block**2,
    disp12MaxDiff=1,
    uniquenessRatio=10,
    speckleWindowSize=100,
    speckleRange=32,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)

right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)

wls = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls.setLambda(8000)
wls.setSigmaColor(1.5)

disp_l = left_matcher.compute(grayL, grayR)
disp_r = right_matcher.compute(grayR, grayL)
disp   = wls.filter(disp_l, rect_left, None, disp_r).astype(np.float32) / 16.0

mask = disp > 1.0

# Visualizacao colorida: azul=longe, vermelho=perto
disp_norm  = cv2.normalize(np.where(mask, disp, 0), None, 0, 255, cv2.NORM_MINMAX)
disp_color = cv2.applyColorMap(np.uint8(disp_norm), cv2.COLORMAP_JET)
disp_color[~mask] = 0

cv2.imwrite("disparity_map.png", disp_color)
cv2.imshow("Disparity Map (azul=longe, vermelho=perto)", disp_color)
cv2.waitKey(0)

# ===========================================================================
# 7. PROFUNDIDADE EM METROS  Z = (f * B) / d
# ===========================================================================

focal    = float(P1[0, 0])
baseline = abs(float(T_rel[0]))  # componente X em metros

print(f"\n[INFO] Focal pos-retificacao: {focal:.2f} px")
print(f"[INFO] Baseline usado: {baseline*100:.2f} cm")

depth = np.where(mask, (focal * baseline) / disp, np.nan)

depth_vis = np.where(np.isnan(depth), 0, np.clip(depth, 0.05, 15.0))
depth_vis = cv2.normalize(depth_vis, None, 0, 255, cv2.NORM_MINMAX)
depth_color = cv2.applyColorMap(np.uint8(depth_vis), cv2.COLORMAP_TURBO)

cv2.imwrite("depth_map.png", depth_color)
cv2.imshow("Depth Map", depth_color)

cy, cx = img_size[1] // 2, img_size[0] // 2
d_center = depth[cy, cx]
if not np.isnan(d_center):
    print(f"[INFO] Profundidade no centro: {d_center:.2f} m")
else:
    print("[AVISO] Centro sem disparidade valida")

cv2.waitKey(0)

# ===========================================================================
# 8. POINT CLOUD .ply
# ===========================================================================

pts3d  = cv2.reprojectImageTo3D(disp, Q)
colors = cv2.cvtColor(rect_left, cv2.COLOR_BGR2RGB)
mask_cloud = mask & (pts3d[:,:,2] > 0.05) & (pts3d[:,:,2] < 15)
verts = pts3d[mask_cloud]
cols  = colors[mask_cloud]

with open("output.ply", 'w') as f:
    f.write(f"ply\nformat ascii 1.0\nelement vertex {len(verts)}\n")
    f.write("property float x\nproperty float y\nproperty float z\n")
    f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
    for v, c in zip(verts, cols):
        f.write(f"{v[0]:.4f} {v[1]:.4f} {v[2]:.4f} {c[0]} {c[1]} {c[2]}\n")

print(f"[OK] Point cloud salva: output.ply ({len(verts)} pontos)")

# ===========================================================================
# RESUMO
# ===========================================================================
print("\n===== RESUMO =====")
print(f"Pixels validos no mapa: {mask.sum()} / {mask.size} ({100*mask.mean():.1f}%)")
if mask.any():
    print(f"Disparidade: {disp[mask].min():.1f} ~ {disp[mask].max():.1f} px")
print("Arquivos gerados: epipolar_check.png, disparity_map.png, depth_map.png, output.ply")

cv2.destroyAllWindows()
