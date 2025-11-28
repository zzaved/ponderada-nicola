#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <cg_interfaces/msg/robot_sensors.hpp>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include <algorithm>

using namespace std;

// ============= ESTRUTURAS =============
struct Posicao {
    int x, y;
    
    bool operator==(const Posicao& outro) const {
        return x == outro.x && y == outro.y;
    }
    
    bool operator!=(const Posicao& outro) const {
        return !(*this == outro);
    }
    
    struct Hash {
        size_t operator()(const Posicao& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
};

struct No {
    Posicao pos;
    int g, h, f;
    
    bool operator>(const No& outro) const {
        return f > outro.f;
    }
};

// Célula do mapa
enum Celula {
    DESCONHECIDO = -1,
    LIVRE = 0,
    BLOQUEADO = 1,
    ROBO = 2,
    ALVO = 3
};

// ============= CLASSE EXPLORADOR =============
class ExploradorAutonomo : public rclcpp::Node {
public:
    ExploradorAutonomo() : Node("explorador_autonomo") {
        // Inicializar posição do robô (sempre começa em 0,0 do nosso mapa local)
        robo_ = {0, 0};
        alvo_encontrado_ = false;
        
        // Inicializar mapa (começamos pequeno e expandimos)
        mapa_min_x_ = 0;
        mapa_min_y_ = 0;
        mapa_max_x_ = 0;
        mapa_max_y_ = 0;
        
        // Marcar posição inicial como livre
        mapa_[{0, 0}] = LIVRE;
        
        // Cliente de movimento
        client_move_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        // Subscriber de sensores
        sub_sensores_ = this->create_subscription<cg_interfaces::msg::RobotSensors>(
            "/culling_games/robot_sensors",
            10,
            std::bind(&ExploradorAutonomo::callbackSensores, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "🗺️  Explorador Autônomo - Parte 2");
        RCLCPP_INFO(this->get_logger(), "Algoritmo: Frontier-Based Exploration + A*");
    }
    
    void callbackSensores(const cg_interfaces::msg::RobotSensors::SharedPtr msg) {
        ultima_leitura_ = *msg;
        sensores_atualizados_ = true;
    }
    
    bool esperarServicos() {
        RCLCPP_INFO(this->get_logger(), "Aguardando serviços...");
        
        if (!client_move_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Serviço /move_command não disponível!");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "✓ Serviços disponíveis!");
        return true;
    }
    
    void atualizarMapaComSensores() {
        // Aguardar sensores
        sensores_atualizados_ = false;
        auto start = this->now();
        while (!sensores_atualizados_ && (this->now() - start).seconds() < 2.0) {
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        if (!sensores_atualizados_) {
            RCLCPP_WARN(this->get_logger(), "Timeout esperando sensores!");
            return;
        }
        
        // Atualizar células ao redor baseado nos sensores
        vector<pair<Posicao, string>> leituras = {
            {{robo_.x, robo_.y - 1}, ultima_leitura_.up},
            {{robo_.x, robo_.y + 1}, ultima_leitura_.down},
            {{robo_.x - 1, robo_.y}, ultima_leitura_.left},
            {{robo_.x + 1, robo_.y}, ultima_leitura_.right},
            {{robo_.x - 1, robo_.y - 1}, ultima_leitura_.up_left},
            {{robo_.x + 1, robo_.y - 1}, ultima_leitura_.up_right},
            {{robo_.x - 1, robo_.y + 1}, ultima_leitura_.down_left},
            {{robo_.x + 1, robo_.y + 1}, ultima_leitura_.down_right}
        };
        
        for (auto& [pos, valor] : leituras) {
            if (valor == "f") {
                if (mapa_.find(pos) == mapa_.end()) {
                    mapa_[pos] = LIVRE;
                }
            } else if (valor == "b") {
                mapa_[pos] = BLOQUEADO;
            } else if (valor == "t") {
                mapa_[pos] = ALVO;
                alvo_ = pos;
                alvo_encontrado_ = true;
                RCLCPP_INFO(this->get_logger(), "🎯 ALVO ENCONTRADO em (%d, %d)!", pos.x, pos.y);
            }
            
            // Atualizar bounds do mapa
            mapa_min_x_ = min(mapa_min_x_, pos.x);
            mapa_min_y_ = min(mapa_min_y_, pos.y);
            mapa_max_x_ = max(mapa_max_x_, pos.x);
            mapa_max_y_ = max(mapa_max_y_, pos.y);
        }
        
        // Marcar posição atual
        mapa_[robo_] = LIVRE;
    }
    
    vector<Posicao> identificarFronteiras() {
        vector<Posicao> fronteiras;
        
        // Fronteira = célula LIVRE adjacente a DESCONHECIDO
        for (auto& [pos, tipo] : mapa_) {
            if (tipo != LIVRE) continue;
            
            // Verificar vizinhos (4 direções principais)
            vector<Posicao> vizinhos = {
                {pos.x, pos.y - 1},
                {pos.x, pos.y + 1},
                {pos.x - 1, pos.y},
                {pos.x + 1, pos.y}
            };
            
            for (auto& viz : vizinhos) {
                if (mapa_.find(viz) == mapa_.end()) {
                    // Vizinho desconhecido = fronteira!
                    fronteiras.push_back(pos);
                    break;
                }
            }
        }
        
        return fronteiras;
    }
    
    int heuristica(const Posicao& a, const Posicao& b) {
        return abs(a.x - b.x) + abs(a.y - b.y);
    }
    
    bool ehValido(const Posicao& pos) {
        auto it = mapa_.find(pos);
        return it != mapa_.end() && (it->second == LIVRE || it->second == ALVO);
    }
    
    vector<string> calcularCaminhoAte(const Posicao& destino) {
        priority_queue<No, vector<No>, greater<No>> abertos;
        unordered_map<Posicao, int, Posicao::Hash> custos;
        unordered_map<Posicao, Posicao, Posicao::Hash> pais;
        
        No inicio = {robo_, 0, heuristica(robo_, destino), heuristica(robo_, destino)};
        abertos.push(inicio);
        custos[robo_] = 0;
        
        vector<Posicao> direcoes = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
        
        while (!abertos.empty()) {
            No atual = abertos.top();
            abertos.pop();
            
            if (atual.pos == destino) {
                return reconstruirCaminho(pais, destino);
            }
            
            for (auto& dir : direcoes) {
                Posicao vizinho = {atual.pos.x + dir.x, atual.pos.y + dir.y};
                
                if (!ehValido(vizinho)) continue;
                
                int novo_custo = custos[atual.pos] + 1;
                
                if (custos.find(vizinho) == custos.end() || novo_custo < custos[vizinho]) {
                    custos[vizinho] = novo_custo;
                    int h = heuristica(vizinho, destino);
                    No novo_no = {vizinho, novo_custo, h, novo_custo + h};
                    abertos.push(novo_no);
                    pais[vizinho] = atual.pos;
                }
            }
        }
        
        return {};  // Sem caminho
    }
    
    vector<string> reconstruirCaminho(const unordered_map<Posicao, Posicao, Posicao::Hash>& pais, 
                                     const Posicao& destino) {
        vector<string> caminho;
        Posicao atual = destino;
        
        while (atual != robo_) {
            Posicao pai = pais.at(atual);
            
            int dx = atual.x - pai.x;
            int dy = atual.y - pai.y;
            
            if (dy == -1) caminho.push_back("up");
            else if (dy == 1) caminho.push_back("down");
            else if (dx == -1) caminho.push_back("left");
            else if (dx == 1) caminho.push_back("right");
            
            atual = pai;
        }
        
        reverse(caminho.begin(), caminho.end());
        return caminho;
    }
    
    bool executarMovimento(const string& direcao) {
        auto request = std::make_shared<cg_interfaces::srv::MoveCmd::Request>();
        request->direction = direcao;
        
        auto future = client_move_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            
            if (response->success) {
                // Atualizar posição do robô
                if (direcao == "up") robo_.y--;
                else if (direcao == "down") robo_.y++;
                else if (direcao == "left") robo_.x--;
                else if (direcao == "right") robo_.x++;
                
                return true;
            }
        }
        
        return false;
    }
    
    void explorar() {
        RCLCPP_INFO(this->get_logger(), "\n=== INICIANDO EXPLORAÇÃO ===");
        
        int iteracao = 0;
        const int MAX_ITERACOES = 1000;  // Limite de segurança
        
        while (!alvo_encontrado_ && iteracao < MAX_ITERACOES) {
            iteracao++;
            
            // 1. Atualizar mapa com sensores
            atualizarMapaComSensores();
            
            RCLCPP_INFO(this->get_logger(), "\n--- Iteração %d ---", iteracao);
            RCLCPP_INFO(this->get_logger(), "Posição atual: (%d, %d)", robo_.x, robo_.y);
            RCLCPP_INFO(this->get_logger(), "Células mapeadas: %zu", mapa_.size());
            
            if (alvo_encontrado_) {
                RCLCPP_INFO(this->get_logger(), "✓ Alvo encontrado! Indo até ele...");
                break;
            }
            
            // 2. Identificar fronteiras
            auto fronteiras = identificarFronteiras();
            
            if (fronteiras.empty()) {
                RCLCPP_WARN(this->get_logger(), "Sem fronteiras! Exploração completa sem encontrar alvo.");
                break;
            }
            
            RCLCPP_INFO(this->get_logger(), "Fronteiras encontradas: %zu", fronteiras.size());
            
            // 3. Escolher fronteira mais próxima
            Posicao melhor_fronteira = fronteiras[0];
            int menor_dist = heuristica(robo_, fronteiras[0]);
            
            for (auto& f : fronteiras) {
                int dist = heuristica(robo_, f);
                if (dist < menor_dist) {
                    menor_dist = dist;
                    melhor_fronteira = f;
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Indo para fronteira: (%d, %d)", 
                       melhor_fronteira.x, melhor_fronteira.y);
            
            // 4. Calcular caminho até fronteira
            auto caminho = calcularCaminhoAte(melhor_fronteira);
            
            if (caminho.empty()) {
                RCLCPP_WARN(this->get_logger(), "Sem caminho para fronteira!");
                continue;
            }
            
            // 5. Executar apenas 1 movimento
            if (executarMovimento(caminho[0])) {
                RCLCPP_INFO(this->get_logger(), "✓ Movimento: %s", caminho[0].c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Falha no movimento!");
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        
        // Se encontrou o alvo, ir até ele
        if (alvo_encontrado_) {
            RCLCPP_INFO(this->get_logger(), "\n=== NAVEGANDO ATÉ O ALVO ===");
            
            auto caminho_final = calcularCaminhoAte(alvo_);
            
            RCLCPP_INFO(this->get_logger(), "Caminho até o alvo: %zu passos", caminho_final.size());
            
            for (size_t i = 0; i < caminho_final.size(); i++) {
                if (executarMovimento(caminho_final[i])) {
                    RCLCPP_INFO(this->get_logger(), "Passo %zu/%zu: %s ✓", 
                               i+1, caminho_final.size(), caminho_final[i].c_str());
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
            
            RCLCPP_INFO(this->get_logger(), "\n🎯 OBJETIVO ALCANÇADO!");
        }
        
        imprimirMapa();
    }
    
    void imprimirMapa() {
        RCLCPP_INFO(this->get_logger(), "\n=== MAPA CONSTRUÍDO ===");
        RCLCPP_INFO(this->get_logger(), "Dimensões: [%d:%d, %d:%d]", 
                   mapa_min_x_, mapa_max_x_, mapa_min_y_, mapa_max_y_);
        
        for (int y = mapa_min_y_; y <= mapa_max_y_; y++) {
            string linha = "";
            for (int x = mapa_min_x_; x <= mapa_max_x_; x++) {
                Posicao p = {x, y};
                
                if (p == robo_) {
                    linha += "R";
                } else if (p == alvo_) {
                    linha += "T";
                } else if (mapa_.find(p) == mapa_.end()) {
                    linha += "?";
                } else if (mapa_[p] == LIVRE) {
                    linha += ".";
                } else if (mapa_[p] == BLOQUEADO) {
                    linha += "#";
                }
            }
            RCLCPP_INFO(this->get_logger(), "%s", linha.c_str());
        }
    }

private:
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_move_;
    rclcpp::Subscription<cg_interfaces::msg::RobotSensors>::SharedPtr sub_sensores_;
    
    unordered_map<Posicao, int, Posicao::Hash> mapa_;
    Posicao robo_, alvo_;
    bool alvo_encontrado_;
    
    int mapa_min_x_, mapa_min_y_, mapa_max_x_, mapa_max_y_;
    
    cg_interfaces::msg::RobotSensors ultima_leitura_;
    bool sensores_atualizados_ = false;
};

// ============= MAIN =============
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto explorador = std::make_shared<ExploradorAutonomo>();
    
    if (explorador->esperarServicos()) {
        explorador->explorar();
    }
    
    rclcpp::shutdown();
    return 0;
}
