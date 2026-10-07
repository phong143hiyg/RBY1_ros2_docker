#!/usr/bin/env bash
set -euo pipefail

readonly COMPOSE_FILE="${COMPOSE_FILE:-docker-compose.yml}"
readonly COMPOSE_PROJECT="${COMPOSE_PROJECT:-rby1-qt-direct}"
readonly SDK_ENDPOINT="${RBY1_SDK_ENDPOINT:-localhost:50051}"

compose=(docker compose -p "${COMPOSE_PROJECT}" -f "${COMPOSE_FILE}")

mapfile -t services < <("${compose[@]}" config --services)
if [[ "${#services[@]}" -ne 1 || "${services[0]}" != "rby1-sim" ]]; then
  printf 'FAIL: compose must contain only rby1-sim; got: %s\n' \
    "${services[*]:-<none>}" >&2
  exit 1
fi

if ! ss -ltnH | awk '{print $4}' | grep -Eq '(^|:)50051$'; then
  printf 'FAIL: simulator SDK endpoint localhost:50051 is not listening\n' >&2
  exit 1
fi

"${compose[@]}" top

docker run --rm --network host \
  local/rby1-sdk-dev:0.10.0 \
  rby1-readonly-smoke "${SDK_ENDPOINT}"

printf 'PASS: compose has one simulator service and the read-only SDK probe succeeded.\n'
