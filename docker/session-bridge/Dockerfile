# Small sidecar that subscribes to webapps' tRPC session.default.subscribe over
# WebSocket and re-exposes the current value as a plain REST GET, so pico4_teleop_controller.py
# can keep using a simple urllib.request call instead of needing a tRPC/WS client itself.
#
# Exists because webapps' real, correctly-built server.js only exposes this data via a
# tRPC subscription (no plain query/REST route) — see docker/xr-pc-service/webapp-server-patch.js,
# which added a REST route for this but was a stale full-server-bundle replacement that broke
# CSS asset resolution (its embedded HTML referenced a newer frontend build's asset hashes
# than what's actually baked into the webapps image). This sidecar gets the same data without
# needing to touch webapps' own server.js at all.
FROM node:22-slim

WORKDIR /app
RUN npm install @trpc/client@11.18.0 ws@8

COPY server.mjs .

ENV WEBAPP_TRPC_WS_URL=ws://webapps:3000/trpc
ENV SESSION_BRIDGE_PORT=9300

CMD ["node", "server.mjs"]
