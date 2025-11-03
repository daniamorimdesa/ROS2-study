# ROS2 Topics: Comunicação entre Nós

## O que é um Topic?
- Um **topic** é um barramento nomeado pelo qual os nós trocam mensagens.
- Usado para fluxos de dados unidirecionais.
- Comunicação anônima: publicadores não sabem quem está assinando, assinantes não sabem quem está publicando.

## Como usar Topics no ROS2
1. Crie um nó (ou use um existente).
2. Dentro do nó, crie publicadores (publishers) e/ou assinantes (subscribers).
3. Para comunicação funcionar:
   - O nome do tópico deve ser igual para publisher e subscriber.
   - O tipo de mensagem deve ser o mesmo.

## Boas práticas
- Nomes de tópicos devem começar com uma letra (ex: `/robot_news`).
- Separe funcionalidades em diferentes nós para modularidade.

## Comandos úteis para tópicos

### Listar tópicos disponíveis
```bash
ros2 topic list
```

### Ver detalhes de um tópico
```bash
ros2 topic info /nome_do_topico
```

### Ver tipo da mensagem de um tópico
```bash
ros2 topic type /nome_do_topico
```

### Escutar mensagens de um tópico
```bash
ros2 topic echo /nome_do_topico
```

### Publicar mensagem manualmente
```bash
ros2 topic pub /nome_do_topico tipo_da_mensagem '{campo: valor}'
```
Exemplo:
```bash
ros2 topic pub /robot_news std_msgs/msg/String "data: 'Olá, mundo!'"
```

### Ver frequência de publicação
```bash
ros2 topic hz /nome_do_topico
```

### Ver largura de banda
```bash
ros2 topic bw /nome_do_topico
```

## Dica
Você pode depurar tópicos usando os comandos acima ou ferramentas gráficas como o `rqt_graph`.

---

Com tópicos, você pode criar aplicações ROS2 modulares e escaláveis, permitindo que diferentes partes do sistema troquem informações de forma eficiente!
