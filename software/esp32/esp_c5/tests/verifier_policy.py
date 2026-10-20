"""Teste les logiques moteurs et servo sur le PC, sans ouvrir de port série."""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile

projet = Path(__file__).resolve().parents[1]
compilateur = shutil.which("g++")
if compilateur is None and Path("C:/msys64/mingw64/bin/g++.exe").is_file():
    compilateur = "C:/msys64/mingw64/bin/g++.exe"
if compilateur is None:
    raise SystemExit("Installer un compilateur C++ g++ pour lancer ce test sur le PC.")

environnement = dict(os.environ)
environnement["PATH"] = str(Path(compilateur).parent) + os.pathsep + environnement.get("PATH", "")

# Edition statique pour MSYS2 sous Windows ; impossible avec le compilateur de macOS.
statique = ["-static"] if os.name == "nt" else []

with tempfile.TemporaryDirectory(prefix="fieldmind-esp-c5-") as temporaire:
    for nom, fichier, options in (
        ("impulsions", "policy_test.cpp", []),
        ("test_moteur", "policy_test.cpp", ["-DFIELDMIND_MOTOR_ONLY=1"]),
        ("main_deux_moteurs", "policy_test.cpp", ["-DFIELDMIND_DUAL_MOTOR=1"]),
        ("test_servo", "servo_test.cpp", []),
        ("angles_centres", "servo_angles_test.cpp", []),
        ("test_tout_moteur", "test_tout_moteur_test.cpp", ["-DFIELDMIND_DUAL_MOTOR=1"]),
        ("pwm_combine", "pwm_combine_test.cpp", ["-I" + str(projet / "tests" / "pwm_stubs")]),
        ("rover3", "rover3_test.cpp", ["-DFIELDMIND_DUAL_MOTOR=1"]),
        ("controleur_complet", "actionneurs_integration_test.cpp", [
            "-DFIELDMIND_DUAL_MOTOR=1", "-DFIELDMIND_ROVER_COMPLET=1",
            "-I" + str(projet / "tests" / "actionneurs_stubs"),
            "-I" + str(projet / "tests" / "pwm_stubs"),
            str(projet / "src" / "actionneurs.cpp"),
        ]),
        ("controleur_test_tout_moteur", "actionneurs_integration_test.cpp", [
            "-DFIELDMIND_DUAL_MOTOR=1",
            "-I" + str(projet / "tests" / "actionneurs_stubs"),
            "-I" + str(projet / "tests" / "pwm_stubs"),
            str(projet / "src" / "actionneurs.cpp"),
        ]),
    ):
        executable = Path(temporaire) / (nom + ".exe")
        subprocess.run(
            [compilateur, "-std=c++17", "-Wall", "-Wextra", "-Werror", *statique,
             *options, str(projet / "tests" / fichier), "-o", str(executable)],
            check=True, env=environnement,
        )
        print(nom + " :", flush=True)
        subprocess.run([str(executable)], check=True, env=environnement)
        if nom.startswith("controleur"):
            for mode in ("echec_tache", "migration", "rover3"):
                subprocess.run([str(executable), mode], check=True, env=environnement)
