#include <rclcpp/rclcpp.hpp>
#include <cg_interfaces/srv/get_map.hpp>
#include <cg_interfaces/srv/move_cmd.hpp>
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

using namespace std;

// ============= ESTRUTURAS =============
struct Posicao {
    int x, y;
    
    bool operator==(const Posicao& outro) const {
        return x == outro.x && y == outro.y;
    }
    
    struct Hash {
        size_t operator()(const Posicao& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
};

struct No {
    Posicao pos;
    int g;  // Custo do início até aqui
    int h;  // Heurística (estimativa até o alvo)
    int f;  // f = g + h
    
    bool operator>(const No& outro) const {
        return f > outro.f;
    }
};

// ============= CLASSE NAVEGADOR =============
class NavegadorAutonomo : public rclcpp::Node {
public:
    NavegadorAutonomo() : Node("navegador_autonomo") {
        // Criar clientes de serviço
        client_get_map_ = this->create_client<cg_interfaces::srv::GetMap>("/get_map");
        client_move_ = this->create_client<cg_interfaces::srv::MoveCmd>("/move_command");
        
        RCLCPP_INFO(this->get_logger(), "Navegador Autônomo iniciado!");
    }
    
    bool esperarServicos() {
        RCLCPP_INFO(this->get_logger(), "Aguardando serviços...");
        
        if (!client_get_map_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Serviço /get_map não disponível!");
            return false;
        }
        
        if (!client_move_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Serviço /move_command não disponível!");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Todos os serviços disponíveis!");
        return true;
    }
    
    bool obterMapa() {
        auto request = std::make_shared<cg_interfaces::srv::GetMap::Request>();
        
        auto future = client_get_map_->async_send_request(request);
        
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            
            // Converter resposta para nosso formato
            altura_ = response->occupancy_grid_shape[0];
            largura_ = response->occupancy_grid_shape[1];
            
            mapa_.resize(altura_, vector<int>(largura_));
            
            for (int i = 0; i < altura_; i++) {
                for (int j = 0; j < largura_; j++) {
                    int idx = i * largura_ + j;
                    string cell = response->occupancy_grid_flattened[idx];
                    
                    if (cell == "f") {
                        mapa_[i][j] = 0;  // livre
                    } else if (cell == "b") {
                        mapa_[i][j] = 1;  // bloqueado
                    } else if (cell == "r") {
                        mapa_[i][j] = 0;  // livre (robô está aqui)
                        robo_ = {j, i};   // [col, row] = [x, y]
                    } else if (cell == "t") {
                        mapa_[i][j] = 0;  // livre (alvo está aqui)
                        alvo_ = {j, i};   // [col, row] = [x, y]
                    }
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Mapa obtido: %dx%d", largura_, altura_);
            RCLCPP_INFO(this->get_logger(), "Robô em (%d, %d)", robo_.x, robo_.y);
            RCLCPP_INFO(this->get_logger(), "Alvo em (%d, %d)", alvo_.x, alvo_.y);
            
            return true;
        }
        
        RCLCPP_ERROR(this->get_logger(), "Falha ao obter mapa!");
        return false;
    }
    
    int heuristica(const Posicao& a, const Posicao& b) {
        return abs(a.x - b.x) + abs(a.y - b.y);
    }
    
    bool ehValido(const Posicao& pos) {
        return pos.x >= 0 && pos.x < largura_ && 
               pos.y >= 0 && pos.y < altura_ && 
               mapa_[pos.y][pos.x] == 0;
    }
    
    vector<string> calcularCaminho() {
        priority_queue<No, vector<No>, greater<No>> abertos;
        unordered_map<Posicao, int, Posicao::Hash> custos;
        unordered_map<Posicao, Posicao, Posicao::Hash> pais;
        
        No inicio = {robo_, 0, heuristica(robo_, alvo_), heuristica(robo_, alvo_)};
        abertos.push(inicio);
        custos[robo_] = 0;
        
        int nos_explorados = 0;
        
        vector<Posicao> direcoes = {{0, -1}, {0, 1}, {-1, 0}, {1, 0}};
        vector<string> nomes_dir = {"up", "down", "left", "right"};
        
        while (!abertos.empty()) {
            No atual = abertos.top();
            abertos.pop();
            nos_explorados++;
            
            if (atual.pos == alvo_) {
                RCLCPP_INFO(this->get_logger(), "Caminho encontrado!");
                RCLCPP_INFO(this->get_logger(), "Nós explorados: %d", nos_explorados);
                return reconstruirCaminho(pais);
            }
            
            for (size_t i = 0; i < direcoes.size(); i++) {
                Posicao vizinho = {atual.pos.x + direcoes[i].x, 
                                  atual.pos.y + direcoes[i].y};
                
                if (!ehValido(vizinho)) continue;
                
                int novo_custo = custos[atual.pos] + 1;
                
                if (custos.find(vizinho) == custos.end() || novo_custo < custos[vizinho]) {
                    custos[vizinho] = novo_custo;
                    int h = heuristica(vizinho, alvo_);
                    No novo_no = {vizinho, novo_custo, h, novo_custo + h};
                    abertos.push(novo_no);
                    pais[vizinho] = atual.pos;
                }
            }
        }
        
        RCLCPP_ERROR(this->get_logger(), "Nenhum caminho encontrado!");
        return {};
    }
    
    vector<string> reconstruirCaminho(const unordered_map<Posicao, Posicao, Posicao::Hash>& pais) {
        vector<string> caminho;
        Posicao atual = alvo_;
        
        while (!(atual == robo_)) {
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
        
        RCLCPP_INFO(this->get_logger(), "Tamanho do caminho: %zu passos", caminho.size());
        
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
            return response->success;
        }
        
        return false;
    }
    
    void executarNavegacao() {
        if (!obterMapa()) {
            RCLCPP_ERROR(this->get_logger(), "Não foi possível obter o mapa!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "\n=== CALCULANDO CAMINHO COM A* ===");
        
        auto caminho = calcularCaminho();
        
        if (caminho.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Não há caminho válido!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "\n=== EXECUTANDO NAVEGAÇÃO ===");
        
        for (size_t i = 0; i < caminho.size(); i++) {
            RCLCPP_INFO(this->get_logger(), "Passo %zu/%zu: %s", 
                       i+1, caminho.size(), caminho[i].c_str());
            
            if (executarMovimento(caminho[i])) {
                RCLCPP_INFO(this->get_logger(), "✓ Sucesso!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "✗ Falha no movimento!");
                return;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        
        RCLCPP_INFO(this->get_logger(), "\n🎯 OBJETIVO ALCANÇADO!");
    }

private:
    rclcpp::Client<cg_interfaces::srv::GetMap>::SharedPtr client_get_map_;
    rclcpp::Client<cg_interfaces::srv::MoveCmd>::SharedPtr client_move_;
    
    vector<vector<int>> mapa_;
    int altura_, largura_;
    Posicao robo_, alvo_;
};

// ============= MAIN =============
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto navegador = std::make_shared<NavegadorAutonomo>();
    
    if (navegador->esperarServicos()) {
        navegador->executarNavegacao();
    }
    
    rclcpp::shutdown();
    return 0;
}
