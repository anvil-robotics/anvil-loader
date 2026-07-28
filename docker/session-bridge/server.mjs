import http from 'node:http';
import { createTRPCClient, createWSClient, wsLink } from '@trpc/client';

const WEBAPP_TRPC_WS_URL = process.env.WEBAPP_TRPC_WS_URL || 'ws://webapps:3000/trpc';
const PORT = process.env.SESSION_BRIDGE_PORT || 9300;

let latest = null;

const wsClient = createWSClient({
  url: WEBAPP_TRPC_WS_URL,
  onOpen: () => console.log(`Connected to ${WEBAPP_TRPC_WS_URL}`),
  onClose: (cause) => console.log('WS closed, will retry:', cause?.reason ?? cause),
});

const trpc = createTRPCClient({
  links: [wsLink({ client: wsClient })],
});

function subscribe() {
  trpc.session.default.subscribe.subscribe(undefined, {
    onData: (data) => {
      latest = data;
      console.log('session.default update:', JSON.stringify(data));
    },
    onError: (err) => {
      console.error('subscription error:', err.message);
    },
    onComplete: () => {
      console.log('subscription completed, retrying in 2s');
      setTimeout(subscribe, 2000);
    },
  });
}
subscribe();

http.createServer((req, res) => {
  if (req.url === '/api/default-session') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify(latest?.data ?? null));
    return;
  }
  res.writeHead(404);
  res.end();
}).listen(PORT, () => console.log(`session-bridge listening on :${PORT}`));
