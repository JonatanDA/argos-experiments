function init()
    log("¡Inicializando e-puck!")
    robot.wheels.set_velocity(10, 10) -- Avanza recto al inicio
end

function step()
    log("Paso de simulación")
    
    -- Verifica si el sensor de proximidad está disponible
    if robot.proximity then
        log("Sensores de proximidad disponibles")
        for i = 1, #robot.proximity do
            log(string.format("Sensor %d: %f", i, robot.proximity[i].value))
            if robot.proximity[i].value > 0.1 then
                log("¡Obstáculo detectado! Girando...")
                robot.wheels.set_velocity(-10, 10) -- Gira a la izquierda
                return
            end
        end
    else
        log("ERROR: 'robot.proximity' no está disponible.")
    end
    
    -- Si no hay obstáculos, sigue avanzando
    robot.wheels.set_velocity(10, 10)
end

function reset()
    log("¡Reset del controlador!")
end

function destroy()
    log("¡Destruyendo controlador!")
end

