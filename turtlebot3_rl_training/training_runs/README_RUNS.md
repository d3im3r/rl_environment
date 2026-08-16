# TurtleBot3 RL Training Runs

## Convención de nombres

Formato:

S##_goalmode_tipo_YYYYMMDD_HHMMSS

Ejemplo:

S01_soft_valid_20260602_203604

Campos:
- S##: Stage del entorno.
- goalmode: single, soft, medium, right, left, separated.
- tipo: valid, eval, ft, bad, test, stable.
- YYYYMMDD_HHMMSS: fecha y hora de la corrida.

---

## S01_single_valid_20260601_185535

Descripción:
Modelo base entrenado en Stage 1 con meta frontal única.

Resultado:
- Stage 1 single validado.
- 10/10 éxitos en evaluación.
- 0 colisiones.
- 0 timeouts.

Checkpoint recomendado:
checkpoints/best_model.pth

Uso:
Base original para transferencia.

---

## S01_soft_valid_20260602_203604

Descripción:
Fine-tuning desde S01_single_valid en Stage 1 con metas soft.

Metas:
- (1.5, 0.0)
- (1.5, 0.25)
- (1.5, -0.25)

Resultado:
- 80/80 éxitos.
- 0 colisiones.
- 0 timeouts.
- Loss estable.

Checkpoint recomendado:
checkpoints/final_model.pth

Uso:
Base actual recomendada para Stage 2.
