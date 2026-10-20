"""Petit client du serveur MCP local de Fusion (hors Fusion, Python 3 standard).

Le port change à chaque redémarrage de Fusion : FUSION_MCP_URL=http://127.0.0.1:<port>/mcp
(à défaut, on cherche le port 27182). Usage :
  python3 fusion_mcp.py run script.py [--ro]     exécute un script Fusion (def run(_context: str))
  python3 fusion_mcp.py shot sortie.png [vue]    capture de la vue courante (vue : current, top, iso-top-right…)
  python3 fusion_mcp.py read '{"queryType": "document", "operation": "open"}'
"""
import base64, json, os, sys, urllib.request

URL = os.environ.get('FUSION_MCP_URL', 'http://127.0.0.1:27182/mcp')


def post(body, sid=None):
    h = {'Content-Type': 'application/json', 'Accept': 'application/json, text/event-stream'}
    if sid:
        h['MCP-Session-Id'] = sid
    r = urllib.request.urlopen(urllib.request.Request(URL, json.dumps(body).encode(), h), timeout=900)
    return r.headers.get('MCP-Session-Id'), r.read().decode()


def call(name, args):
    sid, _ = post({'jsonrpc': '2.0', 'id': 1, 'method': 'initialize',
                   'params': {'protocolVersion': '2025-03-26', 'capabilities': {}, 'clientInfo': {'name': 'claude', 'version': '0'}}})
    try:
        post({'jsonrpc': '2.0', 'method': 'notifications/initialized'}, sid)
    except Exception:
        pass
    raw = post({'jsonrpc': '2.0', 'id': 2, 'method': 'tools/call', 'params': {'name': name, 'arguments': args}}, sid)[1]
    return json.loads(raw)


def main():
    cmd = sys.argv[1]
    if cmd == 'run':
        obj = {'script': open(sys.argv[2], encoding='utf-8').read()}
        if '--ro' in sys.argv:
            obj['readOnly'] = True
        r = call('fusion_mcp_execute', {'featureType': 'script', 'object': obj})
        if 'error' in r:
            print('RPC ERROR', r['error'])
            sys.exit(1)
        for c in r['result'].get('content', []):
            print(c.get('text', c))
        if r['result'].get('isError'):
            sys.exit(2)
    elif cmd == 'shot':
        vue = sys.argv[3] if len(sys.argv) > 3 else 'current'
        r = call('fusion_mcp_read', {'queryType': 'screenshot', 'direction': vue, 'width': 1400, 'height': 900, 'transparentBackground': False})
        for c in r['result']['content']:
            data = c.get('data')
            if c.get('type') == 'text':
                try:
                    data = json.loads(c['text']).get('base64Data')
                except Exception:
                    print(c['text'][:300])
            if data:
                open(sys.argv[2], 'wb').write(base64.b64decode(data))
                print('saved')
    elif cmd == 'read':
        print(json.dumps(call('fusion_mcp_read', json.loads(sys.argv[2])), ensure_ascii=False)[:6000])


if __name__ == '__main__':
    main()
