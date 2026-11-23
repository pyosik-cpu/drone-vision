#!/usr/bin/env bash
set -euo pipefail

# 本脚本必须在 Ubuntu 宿主端运行。
# 用法: ./scripts/run_armor_test.sh <测试图片.jpg> [结果文件名]

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_PATH="${1:-/home/pyosik/桌面/rm.jpg}"
RESULT_NAME="${2:-armor_result.jpg}"
BOARD="root@192.168.42.1"
REMOTE_IMAGE="/root/armor_test.jpg"
REMOTE_RESULT="/root/armor_result.jpg"
REMOTE_MODEL="/root/yolov5_armor_320_bf16.cvimodel"
BINARY="${PROJECT_ROOT}/build/sg2002-release/sg2002_armor_detect"
RESULT_DIR="${PROJECT_ROOT}/results"

if [[ ! -f "${IMAGE_PATH}" ]]; then
  echo "错误：测试图片不存在: ${IMAGE_PATH}" >&2
  exit 1
fi

if [[ ! -x "${BINARY}" ]]; then
  echo "错误：推理程序不存在，请先执行 cmake --build --preset sg2002-release" >&2
  exit 1
fi

mkdir -p "${RESULT_DIR}"

echo "上传推理程序和测试图片……"
scp "${BINARY}" "${BOARD}:/root/sg2002_armor_detect"
scp "${IMAGE_PATH}" "${BOARD}:${REMOTE_IMAGE}"

echo "在 SG2002 上执行推理……"
ssh -t "${BOARD}" \
  "chmod +x /root/sg2002_armor_detect && LD_LIBRARY_PATH=/mnt/system/lib:/mnt/system/usr/lib:/mnt/system/usr/lib/3rd /root/sg2002_armor_detect '${REMOTE_MODEL}' '${REMOTE_IMAGE}' 0.5 0.45 '${REMOTE_RESULT}'"

echo "下载结果到项目目录……"
scp "${BOARD}:${REMOTE_RESULT}" "${RESULT_DIR}/${RESULT_NAME}"

echo "完成：${RESULT_DIR}/${RESULT_NAME}"

