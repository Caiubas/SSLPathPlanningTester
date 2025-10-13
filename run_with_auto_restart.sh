#!/usr/bin/env bash

echo "Iniciando Hades, Caronte, Hermes, Íris, GrSim, Game-Controller e Lcm_Spy"

# Função para reiniciar automaticamente um executável caso ele encerre com erro
run_with_restart() {
    local APP_PATH="$1"
    local APP_NAME="$2"

    cd "$(dirname "$APP_PATH")" || exit 1

    while true; do
        echo "[$(date '+%H:%M:%S')] Iniciando $APP_NAME..."
        ulimit -c unlimited  # permite gerar core dump (não salva, apenas evita bloqueio)

        "./$(basename "$APP_PATH")"
        EXIT_CODE=$?

        if [ $EXIT_CODE -eq 0 ]; then
            echo "[$(date '+%H:%M:%S')] $APP_NAME terminou normalmente (exit 0)."
            break
        else
            if [ $EXIT_CODE -gt 128 ]; then
                SIGNAL=$((EXIT_CODE - 128))
                SIGNAL_NAME="$(kill -l $SIGNAL 2>/dev/null || echo "SIG$SIGNAL")"
                echo "[$(date '+%H:%M:%S')] $APP_NAME caiu com sinal $SIGNAL ($SIGNAL_NAME). Reiniciando em 2s..."
            else
                echo "[$(date '+%H:%M:%S')] $APP_NAME terminou com código $EXIT_CODE. Reiniciando em 2s..."
            fi
            #sleep 2 # Espera 2 segundos antes de reiniciar(se der merda tem que descomentar isso)
        fi
    done
}

# --- Processos controlados (com reinício automático) ---
gnome-terminal --tab --title="Hades" -- bash -c "bash -c '$(declare -f run_with_restart); run_with_restart \"$(pwd)/hades/build/hades\" Hades; exec bash'"
gnome-terminal --tab --title="Caronte" -- bash -c "bash -c '$(declare -f run_with_restart); run_with_restart \"$(pwd)/caronte/build/caronte\" Caronte; exec bash'"
gnome-terminal --tab --title="Hermes" -- bash -c "bash -c '$(declare -f run_with_restart); run_with_restart \"$(pwd)/hermes/build/hermes\" Hermes; exec bash'"

# --- Processos normais (sem reinício automático) ---
gnome-terminal --tab --title="Íris server" -- bash -c "cd iris/web_ui && yarn dev; exec bash"
gnome-terminal --tab --title="Íris web" -- bash -c "cd iris/web_ui/src/backend/lcm_cpp_server/build && ./server; exec bash"
#gnome-terminal --tab --title="Lcm-Spy" -- bash -c "lcm-spy; exec bash"
