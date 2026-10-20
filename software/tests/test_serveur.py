from fastapi.testclient import TestClient

from services.api.serveur import app

client = TestClient(app)


def test_sante() -> None:
    reponse = client.get("/api/sante")
    assert reponse.status_code == 200
    assert reponse.json()["statut"] == "ok"


def test_page_servie_avec_entetes_de_securite() -> None:
    reponse = client.get("/")
    assert reponse.status_code == 200
    assert "FieldMind" in reponse.text
    csp = reponse.headers["Content-Security-Policy"]
    assert "default-src 'self'" in csp
    assert "'sha256-" in csp, "l'import map en ligne doit être autorisée par son empreinte"
    assert reponse.headers["X-Content-Type-Options"] == "nosniff"


def test_three_js_servi_localement() -> None:
    reponse = client.get("/vendor/three/three.module.js")
    assert reponse.status_code == 200
    assert "javascript" in reponse.headers["content-type"]
