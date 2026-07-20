#!/usr/bin/env bash
set -euo pipefail

# Generate Vision Pro-compatible SSL certs for xr teleop + teleimager.
# After running, AirDrop rootCA.pem to Vision Pro and install it as trusted.

HOST_IP="${HOST_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')}"
CERT_DIR="${CERT_DIR:-${HOME}/.config/xr_teleoperate}"

if [[ -z "${HOST_IP}" ]]; then
  echo "Could not detect host IP. Run: HOST_IP=192.168.2.32 $0"
  exit 1
fi

mkdir -p "${CERT_DIR}"
cd "${CERT_DIR}"

echo "Generating certs for host IP: ${HOST_IP}"
echo "Output directory: ${CERT_DIR}"

openssl genrsa -out rootCA.key 2048
openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 3650 \
  -out rootCA.pem -subj "/CN=xr-teleoperate-local"

openssl genrsa -out key.pem 2048
openssl req -new -key key.pem -out server.csr -subj "/CN=${HOST_IP}"

cat > server_ext.cnf <<EOF
subjectAltName = @alt_names
[alt_names]
DNS.1 = localhost
IP.1 = 127.0.0.1
IP.2 = ${HOST_IP}
EOF

openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key \
  -CAcreateserial -out cert.pem -days 3650 -sha256 -extfile server_ext.cnf

chmod 600 key.pem rootCA.key
chmod 644 cert.pem rootCA.pem

cat <<EOF

Done.

Next on Apple Vision Pro:
  1) AirDrop this file to Vision Pro:
       ${CERT_DIR}/rootCA.pem
  2) Open the file on Vision Pro -> Install Profile
  3) Settings -> General -> About -> Certificate Trust Settings
  4) Enable full trust for "xr-teleoperate-local"
  5) Restart Safari, then open:
       https://${HOST_IP}:60001
       https://${HOST_IP}:8012/?ws=wss://${HOST_IP}:8012

Then restart both terminals:
  ./run_sim_xr_teleop.sh
  ./run_xr_teleop.sh
EOF
