curl --location 'http://resisto.energysmartlab.com:8000/drones/mission/result' --header 'Content-Type: application/json' --header 'Authorization: Bearer eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpYXQiOjE2OTUwMjk2ODgsIm5iZiI6MTY5NTAyOTY4OCwiZXhwIjoxNjk1MDI5NzQ4LCJ1c2VyX2lkIjoiNSIsInJvbGVzIjpbImRyb25lc19leHRlcm5hbCJdfQ.GWQy5abV8YoTJ6k29LfjtRW5-NRPjkqxdZKaYg5SU8c' --data '{
"mission_id": 1234,
"resolution_code": 0,
"description": "Nothing to add."
}'

curl --location 'http://10.10.2.5:8000/drones/mission/result' --header 'Content-Type: application/json' --header 'Authorization: Bearer eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpYXQiOjE2OTUwMzM1MjcsIm5iZiI6MTY5NTAzMzUyNywiZXhwIjoxNjk1MDMzNTg3LCJ1c2VyX2lkIjoiNSIsInJvbGVzIjpbImRyb25lc19leHRlcm5hbCJdfQ.O5SPdr54FeSNqSOS6VugxxgSpBJdij_P2m3KoR9uCP8' --data '{
"mission_id": 1234,
"resolution_code": 0,
"description": "Nothing to add."
}'

curl --location 'http://10.10.2.12:8866/token/provide/RESISTO-API' --form 'username="drone"' --form 'password="F1PpE9V!E#Pwz8k53b7b"'

{"access_token":"eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpYXQiOjE2OTUwMzM1MjcsIm5iZiI6MTY5NTAzMzUyNywiZXhwIjoxNjk1MDMzNTg3LCJ1c2VyX2lkIjoiNSIsInJvbGVzIjpbImRyb25lc19leHRlcm5hbCJdfQ.O5SPdr54FeSNqSOS6VugxxgSpBJdij_P2m3KoR9uCP8","token_type":"bearer"}

{"access_token":"eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpYXQiOjE2OTUwMzM3NDUsIm5iZiI6MTY5NTAzMzc0NSwiZXhwIjoxNjk1MDMzODA1LCJ1c2VyX2lkIjoiNSIsInJvbGVzIjpbImRyb25lc19leHRlcm5hbCJdfQ.EGQgwHu3tm-S42XxKKzk_bmuf-flaMN-STkZvTLh4VA","token_type":"bearer"}

curl --location 'http://10.10.2.5:8000/drones/mission/result' --header 'Content-Type: application/json' --header 'Authorization: Bearer eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpYXQiOjE2OTUwMzM3NDUsIm5iZiI6MTY5NTAzMzc0NSwiZXhwIjoxNjk1MDMzODA1LCJ1c2VyX2lkIjoiNSIsInJvbGVzIjpbImRyb25lc19leHRlcm5hbCJdfQ.EGQgwHu3tm-S42XxKKzk_bmuf-flaMN-STkZvTLh4VA' --data '{
"mission_id": 1234,
"resolution_code": 0,
"description": "Nothing to add."
}'

ping -c 30 10.10.2.5 > pingIREC.txt

iperf3 -t 60 -c 10.10.2.5 > iperfIREC.txt
