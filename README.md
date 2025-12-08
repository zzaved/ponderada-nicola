# Navegação Autônoma em Labirintos - ROS 2 (macOS)

[Acesse o vídeo do projeto clicando aqui](https://youtu.be/6haRabsnMYE)

## Objetivo do Projeto

Este projeto implementa dois algoritmos de navegação robótica em labirintos usando ROS 2 Humble no macOS (Apple Silicon):

### Parte 1: Navegação com Mapa Conhecido (A*)
- Robô conhece o mapa completo desde o início
- Utiliza algoritmo A* com heurística Manhattan
- Acessa o serviço ROS `/get_map` para obter o labirinto
- Calcula e executa o caminho ótimo diretamente

### Parte 2: Exploração e Mapeamento (Frontier-Based + A*)
- Robô não conhece o mapa inicialmente
- Usa Frontier-Based Exploration para mapear o labirinto
- Reconstrói o mapa baseado em sensores locais (`/robot_sensors`)
- Aplica A* no mapa construído para encontrar o caminho ótimo
- Executa o caminho mais curto

**Objetivo Educacional**: Demonstrar a diferença entre navegação com conhecimento completo (A* puro) vs. exploração com conhecimento parcial (Frontier-Based + A*).

## Pré-requisitos

- macOS 14+ (Sonoma ou superior)
- Apple Silicon (M1/M2/M3) ou Intel
- Linha de comando (Terminal)

## Instalação

### Git

Verificar instalação:
```bash
git --version
```

Se não estiver instalado:
```bash
xcode-select --install
```

### Nix Package Manager
```bash
# Instalar Nix com suporte a flakes
sh <(curl -L https://nixos.org/nix/install)

# Habilitar flakes (adicionar ao ~/.config/nix/nix.conf)
mkdir -p ~/.config/nix
echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
```

Reinicie o terminal e verifique:
```bash
nix --version
```

### Clonar o Projeto
```bash
# Criar diretório de trabalho
mkdir -p ~/Documents/labirinto_ros2
cd ~/Documents/labirinto_ros2

# Clonar template Nix+Pixi do Gustavo Widman
git clone https://github.com/GustavoWidman/ros2-humble-env.git
cd ros2-humble-env

# Clonar simulador do Professor Nicola
mkdir -p src
cd src
git clone https://github.com/rmnicola/culling_games.git
cd ..
```

### Configurar Ambiente
```bash
# Primeira execução (pode demorar 15-20 minutos)
nix develop

# Dentro do ambiente Nix, adicionar pygame
pixi add pygame

# Compilar pacotes base
colcon build

# Source do workspace
source install/setup.bash
```

### Adicionar Pacote de Navegação
```bash
# Criar pacote C++
cd src
ros2 pkg create --build-type ament_cmake navegador_autonomo --dependencies rclcpp

# Adicionar dependências ao package.xml
cd navegador_autonomo
```

Edite `package.xml` e adicione antes de `</package>`:
```xml
<depend>cg_interfaces</depend>
<depend>std_srvs</depend>
```

Copie os arquivos de implementação fornecidos para `src/`:
- `navegador_parte1.cpp`
- `navegador_parte1_real.cpp`
- `navegador_parte2.cpp`

Atualize `CMakeLists.txt` adicionando antes de `ament_package()`:
```cmake
find_package(cg_interfaces REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(parte1 src/navegador_parte1.cpp)
ament_target_dependencies(parte1 rclcpp)
install(TARGETS parte1 DESTINATION lib/${PROJECT_NAME})

add_executable(parte1_real src/navegador_parte1_real.cpp)
ament_target_dependencies(parte1_real rclcpp cg_interfaces std_srvs)
install(TARGETS parte1_real DESTINATION lib/${PROJECT_NAME})

add_executable(parte2 src/navegador_parte2.cpp)
ament_target_dependencies(parte2 rclcpp cg_interfaces)
install(TARGETS parte2 DESTINATION lib/${PROJECT_NAME})
```

Compile:
```bash
cd ~/Documents/labirinto_ros2/ros2-humble-env
colcon build --packages-select navegador_autonomo
source install/setup.bash
```

## Estrutura do Projeto
```
ros2-humble-env/
├── src/
│   ├── culling_games/          # Simulador (Prof. Nicola)
│   │   ├── cg/                 # Jogo Pygame
│   │   ├── cg_interfaces/      # Mensagens/Serviços ROS
│   │   └── cg_teleop/          # Teleoperação
│   └── navegador_autonomo/     # Implementação
│       ├── src/
│       │   ├── navegador_parte1.cpp       # Teste A* (mapa hardcoded)
│       │   ├── navegador_parte1_real.cpp  # A* com ROS
│       │   └── navegador_parte2.cpp       # Frontier-Based + A*
│       ├── CMakeLists.txt
│       └── package.xml
├── build/                      # Arquivos de build
├── install/                    # Executáveis
├── flake.nix                   # Configuração Nix
└── pixi.toml                   # Dependências Pixi
```

## Como Executar

### Preparação (em CADA novo terminal)
```bash
cd ~/Documents/labirinto_ros2/ros2-humble-env
nix develop
source install/setup.bash
```

### Terminal 1: Iniciar Simulador
```bash
ros2 run cg maze
```

Resultado: Janela Pygame abre mostrando:
- Robô (azul) na posição inicial
- Target (vermelho) na posição final
- Paredes (preto) e espaços livres (branco)

### Terminal 2: Executar Navegação

#### Parte 1 - Teste (mapa hardcoded 10x10)
```bash
ros2 run navegador_autonomo parte1
```

#### Parte 1 - Real (integração com ROS)
```bash
ros2 run navegador_autonomo parte1_real
```

Saída esperada:
```
[INFO] Navegador Autônomo iniciado!
[INFO] Aguardando serviços...
[INFO] Todos os serviços disponíveis!
[INFO] Mapa obtido: 29x29
[INFO] Robô em (X, Y)
[INFO] Alvo em (X, Y)
[INFO] Caminho encontrado!
[INFO] Nós explorados: N
[INFO] Tamanho do caminho: M passos
[INFO] Executando navegação...
[INFO] Passo 1/M: direction
...
[INFO] OBJETIVO ALCANÇADO!
```

#### Parte 2 - Exploração
```bash
ros2 run navegador_autonomo parte2
```

Saída esperada:
```
[INFO] Explorador Autônomo - Parte 2
[INFO] Algoritmo: Frontier-Based Exploration + A*
[INFO] Aguardando serviços...
[INFO] INICIANDO EXPLORAÇÃO
[INFO] --- Iteração 1 ---
[INFO] Posição atual: (X, Y)
[INFO] Células mapeadas: N
[INFO] Fronteiras encontradas: M
[INFO] Indo para fronteira: (X, Y)
...
[INFO] ALVO ENCONTRADO em (X, Y)!
[INFO] NAVEGANDO ATÉ O ALVO
[INFO] Caminho até o alvo: M passos
...
[INFO] OBJETIVO ALCANÇADO!
[INFO] === MAPA CONSTRUÍDO ===
```

## Detalhamento Técnico

### Decisões de Desenvolvimento

Este projeto foi desenvolvido inteiramente em C++ para macOS (Apple Silicon), o que trouxe desafios únicos de compatibilidade com ROS 2. Após tentativas com diferentes abordagens (Docker, Pixi puro), a solução que funcionou foi combinar Nix flakes com Pixi, usando o template desenvolvido pelo Gustavo Widman com sua colaboração direta.

Para a Parte 1, implementei o algoritmo A* clássico com heurística Manhattan, escolha natural para movimento em grid com 4 direções. A integração com os serviços ROS exigiu compreender a estrutura das mensagens, já que o simulador retorna o mapa como array de strings ('f' para livre, 'b' para bloqueado, 'r' para robô, 't' para target) ao invés de inteiros.

Na Parte 2, optei por Frontier-Based Exploration ao invés de Wall Following porque garante exploração completa do espaço. O robô identifica células "fronteira" (livres adjacentes a desconhecidas) e usa A* para navegar até a mais próxima, repetindo até encontrar o alvo. O mapa é construído dinamicamente usando `unordered_map` para eficiência em C++, permitindo que o grid cresça conforme necessário sem pré-alocar memória.

### Algoritmo A* (Parte 1)

- **Arquivo**: `src/navegador_parte1_real.cpp`
- **Complexidade**: O(b^d) onde b = fator de ramificação, d = profundidade
- **Heurística**: Distância Manhattan `|x1-x2| + |y1-y2|`
- **Garantia**: Encontra o caminho ótimo (admissível + consistente)
- **Estrutura de dados**: Priority queue (min-heap)

### Frontier-Based Exploration (Parte 2)

- **Arquivo**: `src/navegador_parte2.cpp`
- **Complexidade**: O(N × A*) onde N = número de iterações
- **Objetivo**: Exploração completa do espaço navegável
- **Estratégia**: Sempre move para a fronteira mais próxima
- **Vantagem**: Garante encontrar o alvo se houver caminho

### Estruturas de Dados
```cpp
// Posição no grid
struct Posicao {
    int x, y;
    struct Hash { ... };  // Para usar em unordered_map
};

// Nó do A*
struct No {
    Posicao pos;
    int g;  // Custo do início
    int h;  // Heurística
    int f;  // f = g + h
};

// Mapa dinâmico (Parte 2)
unordered_map<Posicao, int, Posicao::Hash> mapa_;
// -1 = desconhecido
//  0 = livre
//  1 = bloqueado
//  2 = robô
//  3 = alvo
```

### Serviços e Tópicos ROS

**Serviços:**
- `/get_map` (cg_interfaces/srv/GetMap)
  - Request: vazio
  - Response: `string[] occupancy_grid_flattened`, `uint8[2] occupancy_grid_shape`

- `/move_command` (cg_interfaces/srv/MoveCmd)
  - Request: `string direction` (up/down/left/right)
  - Response: `bool success`, `int8[2] robot_pos`, `int8[2] target_pos`

**Tópicos:**
- `/culling_games/robot_sensors` (cg_interfaces/msg/RobotSensors)
  - Sensores em 8 direções: `up`, `down`, `left`, `right`, `up_left`, `up_right`, `down_left`, `down_right`
  - Valores: 'f' (free), 'b' (blocked), 't' (target)

## Troubleshooting

### Erro: `nix: command not found`

**Causa**: Nix não está instalado ou não está no PATH.

**Solução**:
```bash
sh <(curl -L https://nixos.org/nix/install)
# Reiniciar terminal
```

### Erro: `experimental-features 'nix-command' 'flakes' is disabled`

**Causa**: Flakes não habilitados.

**Solução**:
```bash
mkdir -p ~/.config/nix
echo "experimental-features = nix-command flakes" >> ~/.config/nix/nix.conf
```

### Erro: `No executable found`

**Causa**: Workspace não foi compilado ou sourced corretamente.

**Solução**:
```bash
cd ~/Documents/labirinto_ros2/ros2-humble-env
colcon build --packages-select navegador_autonomo
source install/setup.bash
```

### Erro: Serviços não disponíveis

**Causa**: Simulador não está rodando.

**Verificar serviços**:
```bash
ros2 service list
```

Deve mostrar:
- `/get_map`
- `/move_command`
- `/reset`

**Solução**:
```bash
# Terminal 1
ros2 run cg maze
# Aguardar janela abrir

# Terminal 2 (novo terminal)
cd ~/Documents/labirinto_ros2/ros2-humble-env
nix develop
source install/setup.bash
ros2 run navegador_autonomo parte1_real
```

### Warning: `set_deprecated` durante compilação

**Causa**: Sintaxe de callback antiga no subscriber.

**Solução**: É apenas um warning, não afeta o funcionamento. Pode ignorar.

### Pygame não abre janela

**Causa**: Pygame não instalado no ambiente Pixi.

**Solução**:
```bash
cd ~/Documents/labirinto_ros2/ros2-humble-env
nix develop
pixi add pygame
```

### Limpar build e recompilar
```bash
cd ~/Documents/labirinto_ros2/ros2-humble-env
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## Créditos

- **Simulador Culling Games**: Prof. Rodrigo Nicola ([rmnicola/culling_games](https://github.com/rmnicola/culling_games))
- **Template ROS 2 Humble macOS**: Gustavo Widman ([GustavoWidman/ros2-humble-env](https://github.com/GustavoWidman/ros2-humble-env))
- **Desenvolvimento**: Pablo Azevedo
- **Agradecimentos**: Gustavo Widman pelo suporte na configuração do ambiente macOS

## Autor

**Pablo Azevedo**  
Inteli - Instituto de Tecnologia e Liderança  
2024