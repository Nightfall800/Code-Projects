# Code-Projects intro
Jag vill på denna sida presentera olika program som jag utvecklat dom senaste åren. Har skapat många utav programmen för att jag är väldigt
nyfiken på hur man skapar vissa typer av program. Tack vare AI tekniken har jag kunnat hitta svaren på dom sista frågorna som alltid
hållit en tillbaka.

# FPS version-1.
Så under två år med många kortare luckor av olika anledningar, så skapade jag en 3D motor.
För att göra det enkelt för mig använde jag mig av sprite från Wolfenstein. Detta kallas för 2.5D motor för att det användes sprite.
Jag såg att i "Raylib"(hjälp bibliotek till C/C++), så fanns det många exempel och delar av dom exemplen vart början till 3D motorn.
Jag har fått dörrar, fienden och skjutning att funka. Har även fixat så att 3D modeller går att ladda in. Utvecklad under åren 2024 - 2025.
<img width="800" height="470" alt="image" src="https://github.com/user-attachments/assets/60ef626c-21d0-4408-b633-f5be5027730e" />


# 3D Editorn.
När jag ville testa lite större banor så var jag tvungen att skapa en editor. Att synka flera data-arrays med varandra vart för jobbigt.
Så då gick en massa arbete åt att göra en editor och uppdatera editorn, allt eftersom jag utvecklade min 3D motor.
<img width="800" height="470" alt="image" src="https://github.com/user-attachments/assets/7815a509-facd-4878-b267-3fb8f32aab14" />


# 3D CAD Raylib.
Jag har alltid varit nyfiken på 3D och hur man gör 3D CAD program. Med min kunskap från 3D motorn, så började jag skapa ett 3D CAD program.
Fick många saker att fungera och lärde mig mycket.
<img width="800" height="470" alt="image" src="https://github.com/user-attachments/assets/6b5a8347-0052-49ac-9719-8b37dfde8c27" />


# Korsords editorn.
Fick en tanke i huvudet hur det skulle kunna utföras och ville testa detta praktiskt.


# 3D CAD OpenGL.
Efter att jag utvecklade en 3D CAD med Raylib, så såg jag att mycket kunde förbättras och ljus kunde läggas till.
Jag såg exempel på ImGuizmo, och hur det förenklar hantering av 3D objekt i 3D världen, väldigt mycket.
Jag vet även att OpenGL är vad som används av dom stora företagen när det gäller 3D CAD, så varför inte fortsätta bygga på min kunskap.
Mycket är kvar att göra, men roligt att hålla på med, lite då och då.


# Minesweeper.
Satt och funderade på hur man skulle kunna göra Minesweeper med OpenGL och få en egen version.
Hittade på nätet ett dockument över hur logiken för Minesweeper teoretiskt bör fungera.
Så det löste mycket och mesta av tiden gick till att få till meny och spel grafik.



# 2D CAD.
Har en gång i tiden ägt en version av AutoCAD(Väldigt dyr sådan) och den går inte längre att använda.
För dom låter inte 10 år gamla program få fortsätta. Jag försökte installera det på ny dator, då gamla gav upp.
Så där av är jag väldigt intresserad utav att göra en egen enkel version med dom egenskaper jag behöver.



# FPS version-2.
Höll på och funderade över att porta ett spel som original kod finns till. Så jag tog Quake som kom ut 1996.
Ville att det skulle funka i en modernare OpenGL(som funkar i Windows11). Då tog jag original koden "GL_Quake".
Denna version av OpenGL är anpassad för Windows95, inte för dagens datorer.
Men efter några dagars stympande av kod och fejka original kodens sökvägar, så lessnade jag och kom till
beslutet att göra en egen motor. Vilket innebar att jag använde Quakes pak0.pak-fil, för att läsa ut banan,
alla texture, map lighting och massor av andra finesser. Nu vart det en riktig 3D motor som funkar i Windows11.
I den bifogade bilden så vart texture på fiende fel, men det är nu fixat. Men tycker det är mer bevis på att
detta är en 3D motor som jag byggt och inte foton från ett spel. Har skapat enkla AI på några av fienden.
Skjuta funkar, dörrar och plattformar funkar, map lighting funkar och mycket mer. Att komma till detta stadie
med den kunskap jag har nu, tog tre veckor. Utvecklat under mars-2026.



# PAK-Disassembler.
Jag utvecklade mycket av min 3D motor version 2, utan att veta hur modeller och grafik filer såg ut.
Jag använde några verktyg som jag hittade på internet, men bökiga att använda och man såg inte så mycket som jag ville se.
Jag koncentrerade mig på hur man packar upp filen och kunde då se med olika program, filerna som var i PAK-filen.
Sen gjorde jag även så jag kan packa ihop till samma format. Detta funkar för Quake1, Quake2 och Hexen2.



# PK3-Disassembler.
Av nyfikenhet ville jag se vad som fanns i Quake3 PK3-fil, så jag gjorde ett liknande program som för PAK-filerna.





