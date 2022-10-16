# Feijão com arroz do git 	
	*Obs.: Em alguns momentos o git vai pedir um usuário do github (bote o seu nome de usuário) e uma senha, essa senha não é a senha da sua conta do github! Se trata de uma Personal Acess Token, conversa comigo que a gente cria uma pra você. Não tenha medo do gitola, ele é um google drive dez bilhões de trilhões de vezes melhor ;) .


## Primeira vez que for usar o git
---
> clona o repo que tá no github: <br>

`git clone https://github.com/pedromaquilare/lotusobr `  <br>
> Entra na pasta onde tá o projeto: <br>

`cd lotusobr` <br> 

> Adiciona um repositório remoto, o significado disso é meio complicado, mas você deve fazer isso pra conseguir upar seu código pro github. <br>

`git remote add https://github.com/predomaquilare/lotusobr` <br>	
	
## Comandos do dia a dia

`git pull --rebase origin main ` → "Puxa" todos os arquivos que tão no github pra sua máquina, a gente usa isso pra **sincronizar** o seu pc com o pc de todo mundo. Sempre que for começar a trabalhar usa isso. 

<br>

`git add ./* ` → Adiciona todos os arquivos e subdiretório da pasta ao projeto, sempre que você adicionar um arquivo usa esse comando. 
<br>

--- ---

`git commit -a ` → Serve pra fazer o commit, geralmente esse comando abre um editor de texto, não se assuste, nele você escreve as mudanças que fez, depois de fechar o editor o git vai realizar o commit.
<br>

--- ---

`git push -u origin main `  → Upa o repositório local lá pra o github.
--- ---

<br>
 
  	Pronto, seu código já está no nosso github e todo mundo vai conseguir acessar!! O git tem muitas outras ferramentas massa, precisando a gente vai atraz.	

