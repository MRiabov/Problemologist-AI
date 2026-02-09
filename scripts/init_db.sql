-- Create user if not exists
DO
$do$
BEGIN
   IF NOT EXISTS (
      SELECT FROM pg_catalog.pg_roles
      WHERE  rolname = 'langfuse') THEN

      CREATE ROLE langfuse WITH LOGIN PASSWORD 'langfuse';
   END IF;
END
$do$;

-- Create database if not exists
SELECT 'CREATE DATABASE langfuse OWNER langfuse'
WHERE NOT EXISTS (SELECT FROM pg_database WHERE datname = 'langfuse')\gexec
