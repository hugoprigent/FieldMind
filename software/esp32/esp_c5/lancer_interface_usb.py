"""Serveur local de l'interface Web Serial, sans dependance Python."""
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import argparse
import webbrowser
import socket
from urllib.request import urlopen

class LocalServer(ThreadingHTTPServer):
    allow_reuse_address = False

    def server_bind(self):
        if hasattr(socket, 'SO_EXCLUSIVEADDRUSE'):
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()

def deja_active(url):
    try:
        with urlopen(url, timeout=1) as response:
            existing = response.read(2048).decode('utf-8')
        return 'FieldMind' in existing and 'Pilotage USB' in existing
    except (OSError, UnicodeError):
        return False

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--no-browser', action='store_true')
    parser.add_argument('--port', type=int, default=8767)
    args = parser.parse_args()
    root = Path(__file__).resolve().parent / 'interface_usb'
    handler = partial(SimpleHTTPRequestHandler, directory=str(root))
    url = f'http://127.0.0.1:{args.port}/'
    # Verifier avant bind : Windows peut autoriser plusieurs sockets SO_REUSEADDR.
    if args.port and deja_active(url):
        print(f'Interface deja active : {url}', flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        return
    server = LocalServer(('127.0.0.1', args.port), handler)
    url = f'http://127.0.0.1:{server.server_address[1]}/'
    print(f'Interface USB : {url}', flush=True)
    print('Fermer le moniteur serie. Utiliser Chrome ou Edge. Ctrl+C pour fermer le serveur.', flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

if __name__ == '__main__':
    main()
