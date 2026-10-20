local ActiveLearning = {}

function ActiveLearning.process(obs_ang_offset_global, updated_anterior_indices)
    -- =========================================================================
    -- MODO ESTÁTICO (OBSERVACIÓN PURA)
    -- El robot no realiza ninguna acción de movimiento ni evade obstáculos.
    -- Toda la capacidad de procesamiento se dedica a trackear a los maestros.
    -- =========================================================================

    -- Por seguridad: Si el robot estaba en movimiento (por ejemplo, si se cambió 
    -- el script en caliente), lo detenemos inmediatamente.
    if motion_active then
        motion_active = false
        if r_move then r_move() end
        if robot.wheels then robot.wheels.set_velocity(0, 0) end
        
        -- Si tienes la función de log disponible, avisamos que entramos en reposo
        if change_al_state then
            change_al_state("IDLE", "Modo de Active Learning desactivado. Observación estática.")
        end
    end

    -- Al no hacer nada más, el bucle principal continuará con la FASE G y FASE H
    -- grabando perfectamente las trayectorias de los maestros que pasan por su visión.
end

return ActiveLearning