# ROS2 Services: Comunicação Cliente/Servidor

## O que é um Service?
- Um **service** no ROS2 é um sistema de comunicação do tipo cliente/servidor.
- Pode ser síncrono ou assíncrono.
- Usa um tipo de mensagem para o **Request** e outro para o **Response**.
- Pode ser implementado em Python, C++, etc., diretamente nos nodes.
- **Atenção:** Só pode haver um servidor para cada serviço, mas vários clientes podem chamar esse serviço.

## Características
- Comunicação anônima: o cliente não sabe qual nó é o servidor, e o servidor não sabe quem são os clientes.
- Use verbos para nomear serviços (ex: `add_two_ints`, `enable_button`).

## Como usar Services no ROS2
1. Crie um nó (ou use um existente).
2. Adicione servidores de serviço (cada um com um nome diferente).
3. Para o cliente chamar o serviço, o nome e o tipo do serviço devem ser idênticos ao do servidor.
4. Só pode haver um servidor para um determinado serviço, mas vários clientes.

## Exemplo de Interface de Service
Veja a interface do serviço `AddTwoInts`:
```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```
Saída:
```
int64 a
int64 b
---
int64 sum
```
- `a` e `b` são o request, `sum` é a response.

## Comandos úteis para services

### Ver atalhos e ajuda
```bash
ros2 service
ros2 service -h
```

### Listar todos os services ativos
```bash
ros2 service list
```

### Ver o tipo de um service
```bash
ros2 service type /service_name
```

### Ver a interface do request/response
```bash
ros2 interface show std_srvs/srv/Empty
```

### Chamar um serviço pelo terminal
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
```

### Exemplo com serviço do turtlesim
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, name: 'my_turtle'}"
```

### Testar serviços com interface gráfica
- Use o `rqt` → Plugins → Services → Service Caller

## Dica sobre nomes
- Para mudar o nome de um serviço, use remap no launch ou na linha de comando, mas lembre-se de mudar também no cliente.

## Quando usar Topic ou Service?
- Use **Topic** para enviar dados continuamente, sem esperar resposta.
- Use **Service** quando precisa de uma resposta para uma requisição específica.

---

Com Services, você pode adicionar lógica de requisição e resposta entre seus nós, tornando suas aplicações ROS2 ainda mais poderosas e flexíveis!
