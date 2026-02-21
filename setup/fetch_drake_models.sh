#!/usr/bin/env bash
# Fetch drake_models (same revision as in MODULE.bazel) for use with Pants/runtime.
# Run once; optional. If skipped, set DRAKE_MODELS_PATH at runtime to point to a copy.
set -euo pipefail
REPO="https://github.com/RobotLocomotion/models/archive/69c92595a391eb023c27ab6ac8f80d58a3e4612d.tar.gz"
SHA="ba571c8b369a62c3764d250944b27d72071488789b2494604d23342994141fe2"
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${DIR}/third_party/drake_models"
mkdir -p "${DIR}/third_party"
if [[ -d "${OUT}" ]]; then
  echo "Already exists: ${OUT}"
  exit 0
fi
echo "Fetching drake_models..."
curl -sL "${REPO}" | tar xz -C "${DIR}/third_party"
mv "${DIR}/third_party/models-69c92595a391eb023c27ab6ac8f80d58a3e4612d" "${OUT}"
echo "Done: ${OUT}"
